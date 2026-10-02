package it.unive.pylisa.frontend.expression;

/**
 * Parser for Python numeric literals per PEP 515 (underscore separators) and
 * the standard hex / octal / binary prefixes. Returns a discriminated result so
 * the caller can emit the right LiSA literal type.
 */
public final class PythonNumericLiteral {

	public sealed interface Parsed permits IntegerLit, LongLit, BigIntegerLit, FloatLit, ComplexLit {
	}

	public record IntegerLit(
			int value)
			implements
			Parsed {
	}

	/**
	 * An integer that does not fit 32 bits but fits 64 bits.
	 */
	public record LongLit(
			long value)
			implements
			Parsed {
	}

	/**
	 * An integer that does not fit 64 bits: Python integers are unbounded,
	 * but the analysis does not represent such values.
	 */
	public record BigIntegerLit(
			java.math.BigInteger value)
			implements
			Parsed {
	}

	public record FloatLit(
			double value)
			implements
			Parsed {
	}

	public record ComplexLit(
			double imag)
			implements
			Parsed {
	}

	public static Parsed parse(
			String raw) {
		String s = raw.toLowerCase().replace("_", "");
		if (s.endsWith("j"))
			return new ComplexLit(Double.parseDouble(s.substring(0, s.length() - 1)));
		// prefixed literals are integers, whose digits may include 'e'
		if (!s.startsWith("0x") && !s.startsWith("0o") && !s.startsWith("0b")
				&& (s.contains(".") || s.contains("e")))
			return new FloatLit(Double.parseDouble(s));
		if (s.startsWith("0x"))
			return integer(new java.math.BigInteger(s.substring(2), 16));
		if (s.startsWith("0o"))
			return integer(new java.math.BigInteger(s.substring(2), 8));
		if (s.startsWith("0b"))
			return integer(new java.math.BigInteger(s.substring(2), 2));
		return integer(new java.math.BigInteger(s));
	}

	private static Parsed integer(
			java.math.BigInteger value) {
		if (value.bitLength() < Integer.SIZE)
			return new IntegerLit(value.intValue());
		if (value.bitLength() < Long.SIZE)
			return new LongLit(value.longValue());
		return new BigIntegerLit(value);
	}

	private PythonNumericLiteral() {
	}
}
