package it.unive.pylisa.libraries.lists;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import java.util.HashSet;
import java.util.Set;

/**
 * Native implementation of {@code list.count(self, x)}, the number of the
 * elements equal to {@code x}: an {@code int} between {@code 0} and the length
 * of the list. The elements are compared with {@code ==}: if {@code x} or the
 * elements might be objects (whose {@code __eq__} is not known here), any
 * exception might be raised (see
 * {@link LibrarySpecificationProvider#UNKNOWN_EXCEPTION}).
 */
public class ListCount extends PyNative {

	// whether this is list.index (see ListIndex)
	private final boolean index;

	/**
	 * Builds the native.
	 *
	 * @param cfg      the cfg where the native is called
	 * @param location the location of the call
	 * @param name     the name of the method
	 * @param index    whether the method is {@code list.index}
	 * @param params   the arguments of the call
	 */
	protected ListCount(
			CFG cfg,
			CodeLocation location,
			String name,
			boolean index,
			Expression[] params) {
		super(cfg, location, name, params);
		this.index = index;
	}

	public static ListCount build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListCount(cfg, location, "count", false, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression list = args[0];
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);

		// the types of the compared values
		Set<Type> compared = new HashSet<>(analysis.getRuntimeTypesOf(state, args[1], context, this));
		Set<SymbolicExpression> elements = ListModel.elementsOf(analysis, context, state, list,
				PyClassType.lookup(LibrarySpecificationProvider.LIST), this);
		if (elements == null)
			compared.add(it.unive.lisa.type.Untyped.INSTANCE);
		else
			for (SymbolicExpression element : elements)
				compared.addAll(analysis.getRuntimeTypesOf(state, element, context, this));
		if (compared.isEmpty() || compared.stream().anyMatch(t -> t.isUntyped()
				|| (!PyBinaryDispatch.isBuiltinValueType(t) && !PyBinaryDispatch.fullyKnown(t))))
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.UNKNOWN_EXCEPTION);
		if (index)
			// x might not be in the list (or in the given range)
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.VALUE_ERROR);
		for (int i = 2; i < args.length; i++)
			if (!missing(args[i]) && analysis.getRuntimeTypesOf(state, args[i], context, this).stream()
					.anyMatch(t -> !INT.test(t) && !t.isNullType()))
				raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.TYPE_ERROR);

		// count: 0 <= result <= len, index: 0 <= result < len
		Variable result = new Variable(Int32Type.INSTANCE, "$" + getConstructName() + "@" + loc.getCodeLocation(),
				loc);
		AnalysisState<A> computed = analysis.assign(state, result, new PushAny(Int32Type.INSTANCE, loc), context,
				this);
		computed = analysis.assume(computed, new BinaryExpression(BoolType.INSTANCE,
				new BinaryExpression(BoolType.INSTANCE, new Constant(Int32Type.INSTANCE, 0, loc), result,
						ComparisonLe.INSTANCE, loc),
				new BinaryExpression(BoolType.INSTANCE, result, ListModel.lengthOf(list, loc),
						index ? ComparisonLt.INSTANCE : ComparisonLe.INSTANCE, loc),
				LogicalAnd.INSTANCE, loc), context, this, this);
		if (!computed.getExecutionState().isBottom()) {
			getMetaVariables().add(result);
			computed = computed.withExecutionExpression(result);
		}
		return raising.moveFrom(computed);
	}
}
