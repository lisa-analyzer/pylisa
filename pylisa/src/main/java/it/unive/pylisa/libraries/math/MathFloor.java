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
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.conversions.Floor;
import java.util.Set;

/**
 * Native implementation of {@code math.floor(x)}:
 * <ul>
 * <li>an {@code int} (or a {@code bool}) is returned as an {@code int};</li>
 * <li>a {@code float} is rounded down to an {@code int}, raising
 * {@code ValueError} if it is {@code nan} and {@code OverflowError} if it is
 * infinite;</li>
 * <li>other builtin values (e.g. a {@code str}) raise {@code TypeError};</li>
 * <li>for objects, the result is unknown, as their class might define
 * {@code __floor__}.</li>
 * </ul>
 */
public class MathFloor extends PyNative {

	protected MathFloor(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "floor", params);
	}

	public static MathFloor build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new MathFloor(cfg, location, exprs);
	}

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
		boolean real = unknown || types.stream().anyMatch(t -> t.isNumericType() && !INT.test(t));
		boolean object = unknown || types.stream().anyMatch(t -> !t.isNumericType() && !t.isBooleanType()
				&& !PyBinaryDispatch.isBuiltinValueType(t));
		boolean notNumber = unknown || types.stream().anyMatch(t -> !t.isNumericType() && !t.isBooleanType()
				&& PyBinaryDispatch.isBuiltinValueType(t));

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		UnaryExpression value = new UnaryExpression(Int32Type.INSTANCE, x, Floor.INSTANCE, loc);
		if (!real) {
			if (integral)
				result = result.lub(compute(analysis, context, state, value));
		} else {
			// nan is the only value different from itself
			BinaryExpression isNan = new BinaryExpression(BoolType.INSTANCE, x, x, ComparisonNe.INSTANCE, loc);
			BinaryExpression isInf = new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, x,
							new Constant(Float32Type.INSTANCE, Float.POSITIVE_INFINITY, loc), ComparisonEq.INSTANCE,
							loc),
					new BinaryExpression(BoolType.INSTANCE, x,
							new Constant(Float32Type.INSTANCE, Float.NEGATIVE_INFINITY, loc), ComparisonEq.INSTANCE,
							loc),
					LogicalOr.INSTANCE, loc);
			Satisfiability nan = analysis.satisfies(state, isNan, context, this);
			Satisfiability inf = analysis.satisfies(state, isInf, context, this);
			if (nan != Satisfiability.SATISFIED && inf != Satisfiability.SATISFIED)
				result = result.lub(compute(analysis, context, state, value));
			if (nan != Satisfiability.NOT_SATISFIED)
				raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
			if (inf != Satisfiability.NOT_SATISFIED)
				raise(raising, state, LibrarySpecificationProvider.OVERFLOW_ERROR);
		}

		if (notNumber)
			raise(raising, state, LibrarySpecificationProvider.TYPE_ERROR);
		if (object)
			result = result.lub(unknown(analysis, context, state, Int32Type.INSTANCE));
		return raising.moveFrom(result);
	}
}
