package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import java.util.Set;

/**
 * Native implementation of {@code list.__eq__(self, other)} (and of
 * {@code __ne__}, its negation): a list is equal to another list with the same
 * length whose elements are pairwise equal, and it is not equal to the values
 * that are not lists. The result is {@code False} if the lengths of the lists
 * are known and different, or if some pair of elements is surely different; it
 * is {@code True} if the lengths are known and equal, and all the pairs of
 * elements are surely equal; otherwise, it is unknown. The elements are
 * compared by the value domain: the {@code __eq__} methods of objects are not
 * called.
 */
public class ListEq extends PyNative {

	private final boolean negated;

	protected ListEq(
			CFG cfg,
			CodeLocation location,
			boolean negated,
			Expression[] params) {
		super(cfg, location, negated ? "__ne__" : "__eq__", params);
		this.negated = negated;
	}

	public static ListEq build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListEq(cfg, location, false, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability equal = equal(analysis, context, state, args[0], args[1]);
		if (negated)
			equal = equal.negate();
		if (equal == Satisfiability.SATISFIED || equal == Satisfiability.NOT_SATISFIED)
			return compute(analysis, context, state,
					new Constant(BoolType.INSTANCE, equal == Satisfiability.SATISFIED, loc));
		return unknown(analysis, context, state, BoolType.INSTANCE);
	}

	// whether the list self is equal to other
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability equal(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression self,
			SymbolicExpression other)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, other, context, this);
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this))
			return Satisfiability.UNKNOWN;
		boolean lists = types.stream().anyMatch(ListModel::isList);
		boolean others = types.stream().anyMatch(t -> !ListModel.isList(t));
		Satisfiability result = others ? Satisfiability.NOT_SATISFIED : Satisfiability.BOTTOM;
		if (!lists)
			return result;

		ListModel.Shape left = ListModel.shape(analysis, context, state, self, this);
		ListModel.Shape right = ListModel.shape(analysis, context, state, other, this);
		if (left.length() == null || right.length() == null)
			return result.lub(Satisfiability.UNKNOWN);
		if (!left.length().equals(right.length()))
			return result.lub(Satisfiability.NOT_SATISFIED);

		// pairwise comparison of the elements
		CodeLocation loc = getLocation();
		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		HeapDereference l = new HeapDereference(listType, self, loc);
		HeapDereference r = new HeapDereference(listType, other, loc);
		Satisfiability elements = Satisfiability.SATISFIED;
		for (int i = 0; i < left.length() && elements != Satisfiability.NOT_SATISFIED; i++) {
			Constant index = new Constant(Int32Type.INSTANCE, i, loc);
			AnalysisState<A> le = analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, l, index, loc),
					context, this);
			Satisfiability pair = Satisfiability.BOTTOM;
			for (SymbolicExpression a : le.getExecutionExpressions()) {
				AnalysisState<A> re = analysis.smallStepSemantics(le,
						new AccessChild(Untyped.INSTANCE, r, index, loc), context, this);
				for (SymbolicExpression b : re.getExecutionExpressions())
					pair = pair.lub(analysis.satisfies(re,
							new BinaryExpression(BoolType.INSTANCE, a, b, ComparisonEq.INSTANCE, loc), context, this));
			}
			if (pair == Satisfiability.BOTTOM)
				pair = Satisfiability.UNKNOWN;
			elements = elements.and(pair);
		}
		return result.lub(elements);
	}
}
