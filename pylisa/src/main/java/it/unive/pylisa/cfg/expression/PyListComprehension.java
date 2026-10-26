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
import it.unive.pylisa.libraries.dicts.DictModel;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
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
 * <p>
 * A dict comprehension {@code {key: element for x in iterable ...}} is
 * evaluated in the same way: at each iteration, {@code key} is computed before
 * {@code element}, and the result is either an empty dictionary or one whose
 * entries have the computed keys, that are not tracked (see
 * {@link DictModel#write}), and the computed values.
 */
public class PyListComprehension extends NaryExpression {

	// the number of iterations after which the widening is used
	private static final int WIDENING_THRESHOLD = 3;

	/**
	 * A {@code for x in iterable if c1 if c2 ...} clause.
	 *
	 * @param target     the variable bound to the elements, or the targets they
	 *                       are unpacked into (a {@link TupleCreation} of
	 *                       variables, as in {@code for k, v in pairs})
	 * @param iterable   the iterable
	 * @param conditions the conditions
	 * @param cells      the variables of the target that are shared with
	 *                       lambdas inside the comprehension (see
	 *                       {@link Cells}), from their names in the program to
	 *                       the names of the local variables of the
	 *                       comprehension
	 */
	public record Clause(
			Expression target,
			Expression iterable,
			List<Expression> conditions,
			Map<String, String> cells) {
	}

	// the key of a dict comprehension, or null
	private final Expression key;

	private final Expression element;

	private final List<Clause> clauses;

	// the variable referencing the cells of the comprehension, and their
	// type, if some variable is shared with lambdas inside the comprehension
	private final String cells;

	private final PyClassType cellsType;

	// the variable holding the key computed by an iteration of a dict
	// comprehension
	private final Variable keyVariable;

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
		this(cfg, location, null, element, clauses);
	}

	/**
	 * Builds the dict comprehension, if {@code key} is not {@code null}.
	 *
	 * @param cfg      the cfg where the comprehension is
	 * @param location the location of the comprehension
	 * @param key      the key, or {@code null} for a list comprehension
	 * @param element  the element (the value, for a dict comprehension)
	 * @param clauses  the clauses
	 */
	public PyListComprehension(
			CFG cfg,
			CodeLocation location,
			Expression key,
			Expression element,
			List<Clause> clauses) {
		this(cfg, location, key, element, clauses, null, null);
	}

	/**
	 * Builds the comprehension (a dict comprehension, if {@code key} is not
	 * {@code null}), whose variables might be shared with lambdas inside it
	 * (see {@link Cells}).
	 *
	 * @param cfg       the cfg where the comprehension is
	 * @param location  the location of the comprehension
	 * @param key       the key, or {@code null} for a list comprehension
	 * @param element   the element (the value, for a dict comprehension)
	 * @param clauses   the clauses
	 * @param cells     the local variable referencing the cells of the
	 *                      comprehension, or {@code null} if no variable is
	 *                      shared
	 * @param cellsType the type of the object holding the cells, or
	 *                      {@code null} if no variable is shared
	 */
	public PyListComprehension(
			CFG cfg,
			CodeLocation location,
			Expression key,
			Expression element,
			List<Clause> clauses,
			String cells,
			PyClassType cellsType) {
		super(cfg, location, key == null ? "listcomp" : "dictcomp", subExpressions(key, element, clauses));
		this.key = key;
		this.element = element;
		this.clauses = clauses;
		this.cells = cells;
		this.cellsType = cellsType;
		this.keyVariable = new Variable(Untyped.INSTANCE,
				"$key@" + location.getCodeLocation(), location);
	}

	private static Expression[] subExpressions(
			Expression key,
			Expression element,
			List<Clause> clauses) {
		List<Expression> subs = new ArrayList<>();
		for (Clause clause : clauses) {
			subs.add(clause.target());
			subs.add(clause.iterable());
			subs.addAll(clause.conditions());
		}
		if (key != null)
			subs.add(key);
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
		StringBuilder res = new StringBuilder(key == null ? "[" : "{");
		if (key != null)
			res.append(key).append(": ");
		res.append(element);
		for (Clause clause : clauses) {
			res.append(" for ").append(clause.target()).append(" in ").append(clause.iterable());
			for (Expression condition : clause.conditions())
				res.append(" if ").append(condition);
		}
		return res.append(key == null ? "]" : "}").toString();
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
		if (cells != null)
			// the cells shared with the lambdas inside the comprehension
			start = Cells.allocate(analysis, start, cellsType, cells, loc, this)
					.withExecutionExpressions(start.getExecutionExpressions());

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

		AnalysisState<A> result = key == null ? list(interprocedural, start, body, expressions)
				: dict(interprocedural, start, body, expressions);

		// the variables of the comprehension, and the values of its
		// sub-expressions, are local to it
		Set<Identifier> locals = new HashSet<>();
		for (Clause clause : clauses)
			variables(clause.target(), locals);
		locals.add(keyVariable);
		if (cells != null)
			locals.add(new Variable(Untyped.INSTANCE, cells, loc));
		for (Expression sub : getSubExpressions())
			locals.addAll(sub.getMetaVariables());
		return result.forgetIdentifiers(locals, this);
	}

	// the list: empty if there are no iterations, otherwise with the computed
	// elements at unknown indices
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> list(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> start,
			AnalysisState<A> body,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
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
		getMetaVariables().addAll(creation.getMetaVariables());
		return result;
	}

	// the dictionary: empty if there are no iterations, otherwise with the
	// computed entries
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> dict(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> start,
			AnalysisState<A> body,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		DictionaryCreation creation = new DictionaryCreation(getCFG(), new DerivedLocation(getLocation(),
				"comprehension"));
		AnalysisState<A> result = creation.forwardSemanticsAux(interprocedural, start, new ExpressionSet[0],
				expressions);
		if (!body.getExecutionState().isBottom()) {
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, body, new ExpressionSet[0],
					expressions);
			for (SymbolicExpression dict : created.getExecutionExpressions())
				for (SymbolicExpression value : body.getExecutionExpressions())
					result = result.lub(DictModel.write(analysis, created, dict, keyVariable, value, this)
							.withExecutionExpression(dict));
		} else
			// the errors raised by the iterations
			result = result.lub(body);
		getMetaVariables().addAll(creation.getMetaVariables());
		return result;
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

		// the target is bound to any of the elements of the iterable; the
		// meta variables of the elements (e.g. the receivers of the tuples of
		// d.items()) are forgotten once the target is bound, as each
		// iteration creates new ones (otherwise, the objects of the previous
		// iteration would still be reachable, and their allocation sites
		// would become weak)
		AnalysisState<A> bound = state.bottomExecution();
		for (SymbolicExpression iterable : state.getExecutionExpressions()) {
			Set<Identifier> metas = new HashSet<>();
			AnalysisState<A> elements = Iteration.elements(interprocedural, state, clause.iterable(), iterable, this,
					metas);
			for (SymbolicExpression elem : elements.getExecutionExpressions()) {
				AnalysisState<A> assigned = bind(interprocedural, elements, clause.target(), elem);
				// the values of the variables shared with the lambdas are also
				// visible through their cells
				for (Map.Entry<String, String> cell : clause.cells().entrySet())
					assigned = analysis.assign(assigned, Cells.cell(cells, cell.getKey(), getLocation()),
							new Variable(Untyped.INSTANCE, cell.getValue(), getLocation()), this);
				bound = bound.lub(assigned.forgetIdentifiers(metas, this));
			}
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
		if (key != null) {
			// the key is computed before the value
			AnalysisState<A> evaluated = key.forwardSemantics(bound, interprocedural, expressions);
			expressions.put(key, evaluated);
			AnalysisState<A> assigned = evaluated.bottomExecution();
			for (SymbolicExpression k : evaluated.getExecutionExpressions())
				assigned = assigned.lub(analysis.assign(evaluated, keyVariable, k, this));
			bound = assigned.forgetIdentifiers(key.getMetaVariables(), this);
		}
		AnalysisState<A> computed = element.forwardSemantics(bound, interprocedural, expressions);
		expressions.put(element, computed);
		return computed;
	}

	// binds the target to the value, unpacking it into the variables of a
	// tuple of targets (see Unpacking)
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> bind(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			Expression target,
			SymbolicExpression value)
			throws SemanticException {
		if (target instanceof VariableRef)
			return interprocedural.getAnalysis().assign(state, ((VariableRef) target).getIdentifier(), value, this);
		Expression[] targets = ((TupleCreation) target).getSubExpressions();
		int star = -1;
		for (int i = 0; i < targets.length; i++)
			if (targets[i] instanceof StarExpression)
				star = i;
		return Unpacking.unpack(interprocedural, state, value, targets, star, this,
				(
						s,
						t,
						v) -> bind(interprocedural, s, t, v));
	}

	// the variables of the target
	private static void variables(
			Expression target,
			Set<Identifier> variables) {
		if (target instanceof VariableRef)
			variables.add(((VariableRef) target).getIdentifier());
		else
			for (Expression sub : ((NaryExpression) target).getSubExpressions())
				variables(sub, variables);
	}
}
