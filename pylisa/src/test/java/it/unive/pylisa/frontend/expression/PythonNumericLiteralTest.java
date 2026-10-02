package it.unive.pylisa.frontend.expression;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.math.BigInteger;
import org.junit.jupiter.api.Test;

/**
 * Tests that numeric literals keep the value Python gives them: floats are
 * doubles, and integers of any size are read without overflow.
 */
class PythonNumericLiteralTest {

	@Test
	void floatsAreDoubles() {
		assertEquals(new PythonNumericLiteral.FloatLit(0.1), PythonNumericLiteral.parse("0.1"));
		assertEquals(new PythonNumericLiteral.FloatLit(1e9), PythonNumericLiteral.parse("1e9"));
	}

	@Test
	void integersOfAnySizeAreRead() {
		assertEquals(new PythonNumericLiteral.IntegerLit(2_147_483_647), PythonNumericLiteral.parse("2_147_483_647"));
		assertEquals(new PythonNumericLiteral.LongLit(2_147_483_648L), PythonNumericLiteral.parse("2147483648"));
		assertEquals(new PythonNumericLiteral.BigIntegerLit(BigInteger.ONE.shiftLeft(64)),
				PythonNumericLiteral.parse("18446744073709551616"));
	}

	@Test
	void prefixedIntegersMayContainTheLetterE() {
		assertEquals(new PythonNumericLiteral.IntegerLit(0x1e), PythonNumericLiteral.parse("0x1e"));
		assertEquals(new PythonNumericLiteral.IntegerLit(0xFE), PythonNumericLiteral.parse("0xFE"));
		assertEquals(new PythonNumericLiteral.IntegerLit(5), PythonNumericLiteral.parse("0b101"));
	}
}
