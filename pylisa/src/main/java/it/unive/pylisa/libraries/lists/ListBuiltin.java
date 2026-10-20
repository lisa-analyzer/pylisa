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
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code list(iterable=None)}: {@code list()} is a new
 * empty list, while {@code list(iterable)} is a new list with the elements of
 * {@code iterable}, that are not tracked: they are unknown elements at unknown
 * indices (see {@link ListCreation#UNKNOWN_INDEX}). {@code TypeError} is raised
 * if {@code iterable} might not be iterable (only lists, {@code str} and
 * {@code bytes} are known to be). As for {@code int}, an explicit {@code None}
 * is treated as an omitted argument. The name {@code list} is also the type of
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

		AnalysisState<A> result = state.bottom();
		boolean empty = false, copy = false;
		Satisfiability iterable = Satisfiability.BOTTOM;
		for (SymbolicExpression arg : params[0])
			if (arg instanceof PyNoneConstant)
				empty = true;
			else {
				copy = true;
				iterable = iterable.lub(hasType(analysis, state, arg, ListOperations.LIST.or(STR).or(BYTES)));
			}

		if (empty)
			result = result
					.lub(creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0], expressions));
		if (copy && iterable != Satisfiability.NOT_SATISFIED) {
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
					expressions);
			for (SymbolicExpression list : created.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				result = result.lub(analysis.assign(created, unknown, new PushAny(Untyped.INSTANCE, loc), st)
						.withExecutionExpression(list));
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
