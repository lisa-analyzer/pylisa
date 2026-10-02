package it.unive.pylisa.analysis.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingAdd;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingDiv;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingRem;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingSub;
import it.unive.lisa.symbolic.value.operator.unary.NumericFloor;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.program.PySyntheticLocation;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;
import org.junit.jupiter.api.Test;

/**
 * Tests that {@link ConstantPropagation} computes arithmetic on numeric
 * constants as Python does: integers are unbounded (a result the analysis
 * cannot represent is unknown, never a wrapped-around value) and floats are
 * IEEE doubles.
 */
class ConstantPropagationNumberTest {

	private static final ConstantPropagation DOMAIN = new ConstantPropagation();

	private static final ConstantPropagation TOP = DOMAIN.top();

	@Test
	void integerArithmeticIsExact() throws SemanticException {
		assertEquals(integer(5), eval(NumericNonOverflowingAdd.INSTANCE, integer(2), integer(3)));
		assertEquals(integer(-1), eval(NumericNonOverflowingSub.INSTANCE, integer(2), integer(3)));
		assertEquals(integer(6), eval(NumericNonOverflowingMul.INSTANCE, integer(2), integer(3)));
	}

	@Test
	void integersDoNotWrapAround() throws SemanticException {
		assertEquals(longInteger(4_000_000_000L),
				eval(NumericNonOverflowingMul.INSTANCE, integer(2), integer(2_000_000_000)));
		assertTrue(eval(NumericNonOverflowingMul.INSTANCE, longInteger(Long.MAX_VALUE), integer(2)).isTop());
	}

	@Test
	void floatsAreDoubles() throws SemanticException {
		assertEquals(real(0.1 + 0.2), eval(NumericNonOverflowingAdd.INSTANCE, real(0.1), real(0.2)));
		assertEquals(real(0.1 * 1e9), eval(NumericNonOverflowingMul.INSTANCE, real(0.1), real(1e9)));
		assertEquals(real(2.5), eval(NumericNonOverflowingAdd.INSTANCE, integer(2), real(0.5)));
	}

	@Test
	void trueDivisionAlwaysYieldsAFloat() throws SemanticException {
		assertEquals(real(2.0), eval(NumericNonOverflowingDiv.INSTANCE, integer(6), integer(3)));
		assertEquals(real(0.5), eval(NumericNonOverflowingDiv.INSTANCE, integer(1), integer(2)));
	}

	@Test
	void divisionByZeroHasNoResult() throws SemanticException {
		assertTrue(eval(NumericNonOverflowingDiv.INSTANCE, integer(1), integer(0)).isBottom());
		assertTrue(eval(NumericNonOverflowingDiv.INSTANCE, integer(1), real(0.0)).isBottom());
		assertTrue(eval(NumericNonOverflowingRem.INSTANCE, integer(1), integer(0)).isBottom());
	}

	@Test
	void remainderTakesTheSignOfTheDivisor() throws SemanticException {
		assertEquals(integer(2), eval(NumericNonOverflowingRem.INSTANCE, integer(-7), integer(3)));
		assertEquals(integer(-2), eval(NumericNonOverflowingRem.INSTANCE, integer(7), integer(-3)));
		assertEquals(real(2.5), eval(NumericNonOverflowingRem.INSTANCE, real(-0.5), integer(3)));
	}

	@Test
	void powerIsExactOrUnknown() throws SemanticException {
		assertEquals(integer(1024), eval(Power.INSTANCE, integer(2), integer(10)));
		assertEquals(integer(1), eval(Power.INSTANCE, integer(0), integer(0)));
		assertEquals(integer(-1), eval(Power.INSTANCE, integer(-1), longInteger(4_611_686_018_427_387_905L)));
		assertEquals(real(8.0), eval(Power.INSTANCE, real(2.0), integer(3)));
		assertEquals(real(1.0), eval(Power.INSTANCE, real(0.0), integer(0)));
		// beyond 64 bits, including exponents whose bound would overflow
		assertTrue(eval(Power.INSTANCE, integer(2), integer(100)).isTop());
		assertTrue(eval(Power.INSTANCE, integer(4), longInteger(4_611_686_018_427_387_904L)).isTop());
		// results of the platform's pow: rounding, complex results, errors
		assertTrue(eval(Power.INSTANCE, integer(2), integer(-2)).isTop());
		assertTrue(eval(Power.INSTANCE, integer(2), real(0.5)).isTop());
		assertTrue(eval(Power.INSTANCE, integer(-8), real(1.0 / 3)).isTop());
		assertTrue(eval(Power.INSTANCE, real(0.0), integer(-1)).isTop());
		assertTrue(eval(Power.INSTANCE, real(10.0), integer(400)).isTop());
		assertTrue(eval(Power.INSTANCE, real(Double.NaN), integer(0)).isTop());
	}

	@Test
	void divisionOfLargeIntegersIsUnknown() throws SemanticException {
		assertTrue(eval(NumericNonOverflowingDiv.INSTANCE, longInteger(5_258_986_265_376_043_509L), integer(888_599))
				.isTop());
	}

	@Test
	void zeroRemaindersHaveTheSignOfTheDivisor() throws SemanticException {
		assertEquals(real(0.0), eval(NumericNonOverflowingRem.INSTANCE, real(-4.0), real(2.0)));
		assertEquals(real(-0.0), eval(NumericNonOverflowingRem.INSTANCE, real(4.0), real(-2.0)));
	}

	@Test
	void integersAndFloatsAreEqualOnlyWhenTheirValuesAre() throws SemanticException {
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(ComparisonEq.INSTANCE,
				longInteger(9_007_199_254_740_993L), real(9_007_199_254_740_992.0)));
		assertEquals(Satisfiability.SATISFIED, satisfies(ComparisonEq.INSTANCE, integer(2), real(2.0)));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(ComparisonEq.INSTANCE, real(Double.NaN), real(Double.NaN)));
	}

	@Test
	void orderingsWithNaNAreUndecided() throws SemanticException {
		// deciding them would make both branches of a condition unreachable
		assertEquals(Satisfiability.UNKNOWN, satisfies(PyComparisonLt.INSTANCE, real(Double.NaN), integer(0)));
		assertEquals(Satisfiability.UNKNOWN, satisfies(PyComparisonGe.INSTANCE, real(Double.NaN), integer(0)));
	}

	@Test
	void booleansCountAsIntegers() throws SemanticException {
		assertEquals(integer(2), eval(NumericNonOverflowingAdd.INSTANCE, bool(true), bool(true)));
	}

	@Test
	void negationAndFloorAreExact() throws SemanticException {
		assertEquals(integer(-3), eval(NumericNegation.INSTANCE, integer(3)));
		assertEquals(real(-0.5), eval(NumericNegation.INSTANCE, real(0.5)));
		assertEquals(integer(500_000_000), eval(NumericFloor.INSTANCE, real(0.5 * 1e9)));
		assertEquals(integer(-1), eval(NumericFloor.INSTANCE, real(-0.5)));
		assertEquals(longInteger(5_000_000_000L), eval(NumericFloor.INSTANCE, real(5e9)));
		assertTrue(eval(NumericFloor.INSTANCE, real(Double.NaN)).isTop());
	}

	@Test
	void numericComparisonsAreDecided() throws SemanticException {
		assertEquals(Satisfiability.SATISFIED, satisfies(PyComparisonLt.INSTANCE, real(-0.5), integer(0)));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(PyComparisonLt.INSTANCE, integer(1), integer(0)));
		assertEquals(Satisfiability.SATISFIED, satisfies(PyComparisonGe.INSTANCE, integer(0), real(0.0)));
		assertEquals(bool(true), eval(PyComparisonLt.INSTANCE, integer(1), longInteger(4_000_000_000L)));
		assertEquals(Satisfiability.UNKNOWN, satisfies(PyComparisonLt.INSTANCE, TOP, integer(0)));
	}

	@Test
	void legacySinglePrecisionConstantsAreReadAsDoubles() throws SemanticException {
		ConstantPropagation half = new ConstantPropagation(constant(Float32Type.INSTANCE, 0.5f));
		assertEquals(real(1.5), eval(NumericNonOverflowingAdd.INSTANCE, half, integer(1)));
	}

	@Test
	void unknownOperandsGiveUnknownResults() throws SemanticException {
		assertTrue(eval(NumericNonOverflowingAdd.INSTANCE, TOP, integer(1)).isTop());
		assertTrue(eval(NumericFloor.INSTANCE, TOP).isTop());
	}

	private static ConstantPropagation eval(
			BinaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation right)
			throws SemanticException {
		Constant placeholder = constant(Untyped.INSTANCE, 0);
		return DOMAIN.evalBinaryExpression(
				new BinaryExpression(Untyped.INSTANCE, placeholder, placeholder, operator,
						PySyntheticLocation.INSTANCE),
				left, right, null, null);
	}

	private static ConstantPropagation eval(
			UnaryOperator operator,
			ConstantPropagation argument)
			throws SemanticException {
		Constant placeholder = constant(Untyped.INSTANCE, 0);
		return DOMAIN.evalUnaryExpression(
				new UnaryExpression(Untyped.INSTANCE, placeholder, operator, PySyntheticLocation.INSTANCE),
				argument, null, null);
	}

	private static Satisfiability satisfies(
			BinaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation right)
			throws SemanticException {
		Constant placeholder = constant(Untyped.INSTANCE, 0);
		return DOMAIN.satisfiesBinaryExpression(
				new BinaryExpression(Untyped.INSTANCE, placeholder, placeholder, operator,
						PySyntheticLocation.INSTANCE),
				left, right, null, null);
	}

	private static ConstantPropagation integer(
			int value) {
		return new ConstantPropagation(constant(Int32Type.INSTANCE, value));
	}

	private static ConstantPropagation longInteger(
			long value) {
		return new ConstantPropagation(constant(Int64Type.INSTANCE, value));
	}

	private static ConstantPropagation real(
			double value) {
		return new ConstantPropagation(constant(Float64Type.INSTANCE, value));
	}

	private static ConstantPropagation bool(
			boolean value) {
		return new ConstantPropagation(constant(BoolType.INSTANCE, value));
	}

	private static Constant constant(
			Type type,
			Object value) {
		return new Constant(type, value, PySyntheticLocation.INSTANCE);
	}
}
