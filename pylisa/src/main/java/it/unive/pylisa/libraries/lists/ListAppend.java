package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;

/**
 * Native implementation of {@code list.append(self, x)}, that adds {@code x} at
 * the end of the list and returns {@code None}. Since the length of the list is
 * not tracked, the index of the new element is unknown: {@code x} is added to
 * the elements at unknown indices of the list (see
 * {@link ListCreation#UNKNOWN_INDEX}), joining it with the ones already there
 * (a weak update, as the list might grow any number of times). The elements at
 * known indices are not changed, as appending does not move them. The list
 * surely has an element afterwards (see {@link ListModel#NONEMPTY}).
 */
public class ListAppend extends PyNative {

	protected ListAppend(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "append", params);
	}

	public static ListAppend build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListAppend(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, args[0], loc),
				new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
		// weak update: the new element is joined with the previous ones
		AnalysisState<A> appended = analysis.assign(state, unknown, args[1], context, st).lub(state);
		// the list surely has an element
		appended = ListModel.setNonEmpty(analysis, context, appended, args[0], true, st);
		return compute(analysis, context, appended, new PyNoneConstant(loc));
	}
}
