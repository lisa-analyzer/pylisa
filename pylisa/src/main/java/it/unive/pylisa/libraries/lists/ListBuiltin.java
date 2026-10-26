package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code list([iterable])}: {@code list()} is a new
 * empty list, while {@code list(iterable)} is a new list with the elements of
 * {@code iterable} at unknown indices (see {@link ListCreation#UNKNOWN_INDEX}):
 * any of them (see {@link Iteration#elements}), if {@code iterable} is surely
 * iterable, unknown values otherwise. {@code TypeError} is raised if
 * {@code iterable} might not be iterable (see {@link Iteration#iterable}), e.g.
 * for an explicit {@code None}. The name {@code list} is also the type of
 * lists, e.g. in {@code _sv_verifier.check_type(x, list)}.
 */
public class ListBuiltin extends PyNative {

	protected ListBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "list", params);
	}

	public static ListBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListBuiltin(cfg, location, exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		ListCreation creation = new ListCreation(getCFG(), loc);
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);

		AnalysisState<A> result = state.bottomExecution();
		boolean empty = false, copy = false;
		Satisfiability iterable = Satisfiability.BOTTOM;
		for (SymbolicExpression arg : params[0])
			if (missing(arg))
				empty = true;
			else {
				copy = true;
				iterable = iterable.lub(Iteration.iterable(analysis, state, arg, st));
			}

		if (empty)
			result = result
					.lub(creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0], expressions));
		if (copy && iterable != Satisfiability.NOT_SATISFIED) {
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
					expressions);
			// the elements of the iterable, if it is surely iterable (computed
			// in the state where the list has been created, as they might be
			// new objects, e.g. the tuples of d.items())
			AnalysisState<A> elements = iterable == Satisfiability.SATISFIED && params[0].elements().size() == 1
					? Iteration.elements(interprocedural, created, getSubExpressions()[0], params[0].iterator().next(),
							st,
							getMetaVariables())
					: analysis.smallStepSemantics(created, new PushAny(Untyped.INSTANCE, loc), st);
			for (SymbolicExpression list : created.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				for (SymbolicExpression element : elements.getExecutionExpressions())
					result = result.lub(analysis.assign(elements, unknown, element, st).withExecutionExpression(list));
			}
		}
		if (copy && iterable != Satisfiability.SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		getMetaVariables().addAll(creation.getMetaVariables());
		return result;
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// never invoked: forwardSemanticsAux computes the result
		throw new SemanticException("list is evaluated by forwardSemanticsAux");
	}
}
