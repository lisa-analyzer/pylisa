package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code str.split(sep=None, maxsplit=-1)}: a new list
 * of {@code str}s, whose elements and length are not tracked (they are at
 * unknown indices, see {@link ListCreation#UNKNOWN_INDEX}). {@code sep} must be
 * a {@code str} or {@code None}, and {@code maxsplit} an {@code int}
 * ({@code TypeError} is raised otherwise); an empty separator raises
 * {@code ValueError}.
 */
public class StrSplit extends PyNative {

	protected StrSplit(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "split", params);
	}

	public static StrSplit build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrSplit(cfg, location, exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		// the new list, with strs at unknown indices: the state of the
		// arguments is the one where it has been created, whose computed
		// expressions are the list (see semantics)
		ListCreation creation = new ListCreation(getCFG(), loc);
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		AnalysisState<A> filled = state.bottomExecution();
		for (SymbolicExpression list : created.getExecutionExpressions()) {
			AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, list, loc),
					new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
			filled = filled.lub(analysis.assign(created, unknown, new PushAny(StringType.INSTANCE, loc), context, st)
					.withExecutionExpression(list));
		}
		return super.forwardSemanticsAux(interprocedural, filled, params, expressions);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression sep = args[1], maxsplit = args[2];
		Satisfiability typed = hasType(analysis, context, state, sep, STR.or(NONE));
		if (!missing(maxsplit))
			typed = typed.and(hasType(analysis, context, state, maxsplit, INT));

		// an empty separator
		Satisfiability empty = hasType(analysis, context, state, sep, STR) == Satisfiability.NOT_SATISFIED
				? Satisfiability.NOT_SATISFIED
				: analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, sep,
						new Constant(StringType.INSTANCE, "", loc), ComparisonEq.INSTANCE, loc), context, st);

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (empty != Satisfiability.SATISFIED)
			// the computed expressions of the state are the new list
			result = result.lub(state);
		if (empty != Satisfiability.NOT_SATISFIED)
			raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
		return raising.moveFrom(typeChecked(state, typed, result, raising));
	}
}
