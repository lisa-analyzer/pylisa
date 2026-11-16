package it.unive.pylisa.libraries.math;

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
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import java.util.Set;

/**
 * The native implementation of the functions of {@code math} computing a
 * {@code float} from a number {@code x}:
 * <ul>
 * <li>for an {@code int}, a {@code bool} or a {@code float}, the result is a
 * {@code float}, whose value is not tracked; {@code ValueError} is raised when
 * {@code x} is outside the domain of the function (see {@link #outsideDomain}),
 * and an {@code int} too large to be converted to a {@code float} raises
 * {@code OverflowError} (which is not raised if it is known to be an
 * {@code int} of 32 bits);</li>
 * <li>other builtin values (e.g. a {@code str}) raise {@code TypeError};</li>
 * <li>for objects, the result is unknown, as their class might define
 * {@code __float__} (as for {@link MathFloor}).</li>
 * </ul>
 */
public abstract class MathFloatFunction extends PyNative {

	/**
	 * Builds the native.
	 *
	 * @param cfg      the cfg where the native is called
	 * @param location the location of the call
	 * @param name     the name of the function
	 * @param params   the arguments of the call
	 */
	protected MathFloatFunction(
			CFG cfg,
			CodeLocation location,
			String name,
			Expression[] params) {
		super(cfg, location, name, params);
	}

	/**
	 * The condition holding when {@code x} is outside the domain of the
	 * function, raising {@code ValueError}.
	 *
	 * @param x   the argument
	 * @param loc the location of the call
	 *
	 * @return the condition
	 */
	protected abstract SymbolicExpression outsideDomain(
			SymbolicExpression x,
			CodeLocation loc);

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression x = args[0];
		Set<Type> types = analysis.getRuntimeTypesOf(state, x, context, this);
		boolean unknown = types.isEmpty() || types.stream().anyMatch(Type::isUntyped);
		boolean integral = unknown || types.stream().anyMatch(INT);
		boolean number = unknown || types.stream().anyMatch(t -> t.isNumericType() || t.isBooleanType());
		boolean object = unknown || types.stream().anyMatch(t -> !t.isNumericType() && !t.isBooleanType()
				&& !PyBinaryDispatch.isBuiltinValueType(t));
		boolean notNumber = unknown || types.stream().anyMatch(t -> !t.isNumericType() && !t.isBooleanType()
				&& PyBinaryDispatch.isBuiltinValueType(t));

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (number) {
			Satisfiability outside = analysis.satisfies(state, outsideDomain(x, loc), context, this);
			if (outside != Satisfiability.SATISFIED)
				result = result.lub(compute(analysis, context, state, new PushAny(Float32Type.INSTANCE, loc)));
			if (outside != Satisfiability.NOT_SATISFIED)
				raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
			if (integral && analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, x,
							new Constant(Int32Type.INSTANCE, Integer.MIN_VALUE, loc), ComparisonGe.INSTANCE, loc),
					new BinaryExpression(BoolType.INSTANCE, x,
							new Constant(Int32Type.INSTANCE, Integer.MAX_VALUE, loc), ComparisonLe.INSTANCE, loc),
					LogicalAnd.INSTANCE, loc), context, this) != Satisfiability.SATISFIED)
				raise(raising, state, LibrarySpecificationProvider.OVERFLOW_ERROR);
		}

		if (notNumber)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		if (object)
			result = result.lub(unknown(analysis, context, state, null));
		return raising.moveFrom(result);
	}
}
