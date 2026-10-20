package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * A list comprehension {@code [element for x in iterable if condition ...]}.
 * Its value is a new list, whose elements are the values of {@code element}
 * computed while {@code x} is bound to each of the elements of {@code iterable}
 * for which the conditions hold. The iterations are summarized: {@code x} is
 * bound to any of the elements of {@code iterable}, and the effects of the
 * iterations are computed up to a fixpoint. The result is either an empty list
 * (when there are no iterations), or a list whose elements are the computed
 * values, at unknown indices (as the number of iterations is not known). The
 * variables of the comprehension are local to it (they are renamed by the
 * parser).
 */
public class PyListComprehension extends NaryExpression {

	// the number of iterations after which the widening is used
	private static final int WIDENING_THRESHOLD = 3;

	/**
	 * A {@code for x in iterable if c1 if c2 ...} clause.
	 *
	 * @param target     the variable bound to the elements
	 * @param iterable   the iterable
	 * @param conditions the conditions
	 */
	public record Clause(
			VariableRef target,
			Expression iterable,
			List<Expression> conditions) {
	}

	private final Expression element;

	private final List<Clause> clauses;

	/**
	 * Builds the comprehension.
	 *
	 * @param cfg      the cfg where the comprehension is
	 * @param location the location of the comprehension
	 * @param element  the element
	 * @param clauses  the clauses
	 */
	public PyListComprehension(
			CFG cfg,
			CodeLocation location,
			Expression element,
			List<Clause> clauses) {
		super(cfg, location, "listcomp", subExpressions(element, clauses));
		this.element = element;
		this.clauses = clauses;
	}

	private static Expression[] subExpressions(
			Expression element,
			List<Clause> clauses) {
		List<Expression> subs = new ArrayList<>();
		for (Clause clause : clauses) {
			subs.add(clause.target());
			subs.add(clause.iterable());
			subs.addAll(clause.conditions());
		}
		subs.add(element);
		return subs.toArray(Expression[]::new);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		StringBuilder res = new StringBuilder("[").append(element);
		for (Clause clause : clauses) {
			res.append(" for ").append(clause.target()).append(" in ").append(clause.iterable());
			for (Expression condition : clause.conditions())
				res.append(" if ").append(condition);
		}
		return res.append("]").toString();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();

		// the first iterable is evaluated once, before the iterations
		Expression first = clauses.get(0).iterable();
		AnalysisState<A> start = first.forwardSemantics(entryState, interprocedural, expressions);
		expressions.put(first, start);

		// the states before the iterations, up to a fixpoint
		AnalysisState<A> before = start;
		AnalysisState<A> body = iterate(interprocedural, before, 0, expressions);
		for (int i = 0;; i++) {
			AnalysisState<A> next = before
					.lub(body.withExecutionExpressions(before.getExecutionExpressions()));
			if (next.lessOrEqual(before))
				break;
			before = i < WIDENING_THRESHOLD ? next : before.widening(next);
			body = iterate(interprocedural, before, 0, expressions);
		}

		// the list: empty if there are no iterations, otherwise with the
		// computed elements at unknown indices
		ListCreation creation = new ListCreation(getCFG(), new DerivedLocation(loc, "comprehension"));
		AnalysisState<A> result = creation.forwardSemanticsAux(interprocedural, start, new ExpressionSet[0],
				expressions);
		if (!body.getExecutionState().isBottom()) {
			Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, body, new ExpressionSet[0],
					expressions);
			for (SymbolicExpression list : created.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX,
								loc),
						loc);
				for (SymbolicExpression value : body.getExecutionExpressions())
					result = result.lub(analysis.assign(created, unknown, value, this).withExecutionExpression(list));
			}
		} else
			// the errors raised by the iterations
			result = result.lub(body);

		// the variables of the comprehension, and the values of its
		// sub-expressions, are local to it
		Set<Identifier> locals = new HashSet<>();
		for (Clause clause : clauses)
			locals.add(clause.target().getVariable());
		for (Expression sub : getSubExpressions())
			locals.addAll(sub.getMetaVariables());
		getMetaVariables().addAll(creation.getMetaVariables());
		return result.forgetIdentifiers(locals, this);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// never invoked: forwardSemantics evaluates the sub-expressions
		throw new SemanticException("List comprehensions are evaluated by forwardSemantics");
	}

	// one iteration of the clauses from the i-th one, starting from the state
	// where the i-th iterable has been evaluated: the result is the state where
	// the element has been computed
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> iterate(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			int i,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Clause clause = clauses.get(i);

		// the target is bound to any of the elements of the iterable
		AnalysisState<A> bound = state.bottomExecution();
		for (SymbolicExpression iterable : state.getExecutionExpressions()) {
			AnalysisState<A> elements = Iteration.elements(analysis, state, clause.iterable(), iterable, this);
			for (SymbolicExpression elem : elements.getExecutionExpressions())
				bound = bound.lub(analysis.assign(elements, clause.target().getVariable(), elem, this));
			// e.g. the TypeError raised by iterating an int
			bound = bound.lub(elements.bottomExecution());
		}

		// the conditions hold
		for (Expression condition : clause.conditions()) {
			if (bound.getExecutionState().isBottom())
				break;
			AnalysisState<A> evaluated = condition.forwardSemantics(bound, interprocedural, expressions);
			expressions.put(condition, evaluated);
			AnalysisState<A> holds = evaluated.bottomExecution();
			for (SymbolicExpression cond : evaluated.getExecutionExpressions())
				holds = holds.lub(analysis.assume(evaluated, cond, condition, this));
			bound = holds.forgetIdentifiers(condition.getMetaVariables(), this);
		}
		if (bound.getExecutionState().isBottom())
			return bound;

		if (i + 1 < clauses.size()) {
			// the next clause, iterating for each element of this one
			Expression next = clauses.get(i + 1).iterable();
			AnalysisState<A> evaluated = next.forwardSemantics(bound, interprocedural, expressions);
			expressions.put(next, evaluated);
			return iterate(interprocedural, evaluated, i + 1, expressions);
		}
		AnalysisState<A> computed = element.forwardSemantics(bound, interprocedural, expressions);
		expressions.put(element, computed);
		return computed;
	}
}
