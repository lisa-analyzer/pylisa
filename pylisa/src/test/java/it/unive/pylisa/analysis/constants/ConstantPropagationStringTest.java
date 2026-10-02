package it.unive.pylisa.analysis.constants;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.StringConcat;
import it.unive.lisa.symbolic.value.operator.binary.StringContains;
import it.unive.lisa.symbolic.value.operator.binary.StringEndsWith;
import it.unive.lisa.symbolic.value.operator.binary.StringEquals;
import it.unive.lisa.symbolic.value.operator.binary.StringMatches;
import it.unive.lisa.symbolic.value.operator.binary.StringStartsWith;
import it.unive.lisa.symbolic.value.operator.binary.StringSubstringToEnd;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.program.PySyntheticLocation;
import org.junit.jupiter.api.Test;

/**
 * Tests the exact evaluation of string operations and equality comparisons on
 * constants performed by {@link ConstantPropagation}: when both operands are
 * known constants, the result must be the concrete result; when an operand is
 * unknown, nothing may be claimed.
 */
class ConstantPropagationStringTest {

	private static final ConstantPropagation DOMAIN = new ConstantPropagation();

	private static final ConstantPropagation TOP = DOMAIN.top();

	@Test
	void concatenationOfTwoStringsIsExact() throws SemanticException {
		assertEquals(string("ab"), eval(StringConcat.INSTANCE, string("a"), string("b")));
	}

	@Test
	void substringToEndDropsThePrefix() throws SemanticException {
		assertEquals(string("/x"), eval(StringSubstringToEnd.INSTANCE, string("~/x"), integer(1)));
	}

	@Test
	void substringToEndOutOfBoundsIsUnknown() throws SemanticException {
		assertTrue(eval(StringSubstringToEnd.INSTANCE, string("ab"), integer(5)).isTop());
	}

	@Test
	void stringPredicatesAreDecidedOnConstants() throws SemanticException {
		assertEquals(Satisfiability.SATISFIED, satisfies(StringStartsWith.INSTANCE, string("/a"), string("/")));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(StringStartsWith.INSTANCE, string("a"), string("/")));
		assertEquals(Satisfiability.SATISFIED, satisfies(StringEndsWith.INSTANCE, string("a/"), string("/")));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(StringEndsWith.INSTANCE, string("a"), string("/")));
		assertEquals(Satisfiability.SATISFIED, satisfies(StringContains.INSTANCE, string("a//b"), string("//")));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(StringContains.INSTANCE, string("a/b"), string("//")));
		assertEquals(Satisfiability.SATISFIED, satisfies(StringEquals.INSTANCE, string("a"), string("a")));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(StringEquals.INSTANCE, string("a"), string("b")));
		assertEquals(Satisfiability.SATISFIED, satisfies(StringMatches.INSTANCE, string("abc"), string("[a-z]+")));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(StringMatches.INSTANCE, string("1bc"), string("[a-z]+")));
	}

	@Test
	void stringPredicatesAreEvaluatedToBooleans() throws SemanticException {
		assertEquals(bool(true), eval(StringStartsWith.INSTANCE, string("/a"), string("/")));
		assertEquals(bool(false), eval(StringEquals.INSTANCE, string("a"), string("b")));
	}

	@Test
	void equalityFollowsPythonSemantics() throws SemanticException {
		assertEquals(Satisfiability.SATISFIED, satisfies(ComparisonEq.INSTANCE, string("a"), string("a")));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(ComparisonEq.INSTANCE, string("a"), string("b")));
		assertEquals(Satisfiability.SATISFIED, satisfies(ComparisonEq.INSTANCE, integer(2), integer(2)));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(ComparisonEq.INSTANCE, string("2"), integer(2)));
		assertEquals(Satisfiability.SATISFIED, satisfies(ComparisonEq.INSTANCE, bool(true), integer(1)));
		assertEquals(Satisfiability.NOT_SATISFIED, satisfies(ComparisonNe.INSTANCE, string("a"), string("a")));
		assertEquals(bool(true), eval(ComparisonEq.INSTANCE, string("a"), string("a")));
		assertEquals(bool(true), eval(ComparisonNe.INSTANCE, string("a"), string("b")));
	}

	@Test
	void unknownOperandsDecideNothing() throws SemanticException {
		assertTrue(eval(StringConcat.INSTANCE, TOP, string("b")).isTop());
		assertTrue(eval(ComparisonEq.INSTANCE, string("a"), TOP).isTop());
		assertEquals(Satisfiability.UNKNOWN, satisfies(StringStartsWith.INSTANCE, TOP, string("/")));
		assertEquals(Satisfiability.UNKNOWN, satisfies(ComparisonEq.INSTANCE, TOP, string("a")));
	}

	@Test
	void booleanConstantsAreDecided() throws SemanticException {
		assertEquals(Satisfiability.SATISFIED, DOMAIN.satisfiesAbstractValue(bool(true), null, null));
		assertEquals(Satisfiability.NOT_SATISFIED, DOMAIN.satisfiesAbstractValue(bool(false), null, null));
		assertEquals(Satisfiability.UNKNOWN, DOMAIN.satisfiesAbstractValue(TOP, null, null));
		assertEquals(Satisfiability.SATISFIED,
				DOMAIN.satisfiesConstant(constant(BoolType.INSTANCE, true), null, null));
		assertEquals(Satisfiability.NOT_SATISFIED,
				DOMAIN.satisfiesConstant(constant(BoolType.INSTANCE, false), null, null));
	}

	@Test
	void conditionsFollowPythonTruthiness() throws SemanticException {
		assertEquals(Satisfiability.SATISFIED,
				DOMAIN.satisfiesConstant(constant(StringType.INSTANCE, "a"), null, null));
		assertEquals(Satisfiability.NOT_SATISFIED,
				DOMAIN.satisfiesConstant(constant(StringType.INSTANCE, ""), null, null));
		assertEquals(Satisfiability.NOT_SATISFIED, DOMAIN.satisfiesAbstractValue(integer(0), null, null));
		assertEquals(Satisfiability.SATISFIED, DOMAIN.satisfiesAbstractValue(integer(-2), null, null));
		assertEquals(Satisfiability.NOT_SATISFIED,
				DOMAIN.satisfiesConstant(new it.unive.pylisa.symbolic.PyNoneConstant(PySyntheticLocation.INSTANCE),
						null, null));
	}

	@Test
	void assumingAFalseComparisonMakesTheStateUnreachable() throws SemanticException {
		Variable x = new Variable(StringType.INSTANCE, "x", PySyntheticLocation.INSTANCE);
		ValueEnvironment<ConstantPropagation> env = DOMAIN.makeLattice().putState(x, string("a"));

		ValueEnvironment<ConstantPropagation> impossible = DOMAIN.assumeBinaryExpression(env,
				comparison(ComparisonEq.INSTANCE, x, constant(StringType.INSTANCE, "b")), null, null,
				UninformedOracle.INSTANCE);
		ValueEnvironment<ConstantPropagation> possible = DOMAIN.assumeBinaryExpression(env,
				comparison(ComparisonEq.INSTANCE, x, constant(StringType.INSTANCE, "a")), null, null,
				UninformedOracle.INSTANCE);

		assertTrue(impossible.isBottom());
		assertFalse(possible.isBottom());
		assertEquals(env, possible);
	}

	private static ConstantPropagation eval(
			BinaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation right)
			throws SemanticException {
		return DOMAIN.evalBinaryExpression(expression(operator), left, right, null, null);
	}

	private static Satisfiability satisfies(
			BinaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation right)
			throws SemanticException {
		return DOMAIN.satisfiesBinaryExpression(expression(operator), left, right, null, null);
	}

	private static BinaryExpression expression(
			BinaryOperator operator) {
		Constant placeholder = constant(Untyped.INSTANCE, 0);
		return comparison(operator, placeholder, placeholder);
	}

	private static BinaryExpression comparison(
			BinaryOperator operator,
			it.unive.lisa.symbolic.value.ValueExpression left,
			it.unive.lisa.symbolic.value.ValueExpression right) {
		return new BinaryExpression(Untyped.INSTANCE, left, right, operator, PySyntheticLocation.INSTANCE);
	}

	private static ConstantPropagation string(
			String value) {
		return new ConstantPropagation(constant(StringType.INSTANCE, value));
	}

	private static ConstantPropagation integer(
			int value) {
		return new ConstantPropagation(constant(Int32Type.INSTANCE, value));
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
