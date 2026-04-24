package it.unive.pylisa.frontend.expression;

/**
 * Parser for Python numeric literals per PEP 515 (underscore separators) and
 * the standard hex / octal / binary prefixes. Returns a discriminated result so
 * the caller can emit the right LiSA literal type.
 */
public final class PythonNumericLiteral {

	public sealed interface Parsed permits IntegerLit, FloatLit, ComplexLit {
	}

	public record IntegerLit(
			int value)
			implements
			Parsed {
	}

	public record FloatLit(
			float value)
			implements
			Parsed {
	}

	public record ComplexLit(
			float imag)
			implements
			Parsed {
	}

	public static Parsed parse(
			String raw) {
		String s = raw.toLowerCase().replace("_", "");
		if (s.endsWith("j"))
			return new ComplexLit(Float.parseFloat(s.substring(0, s.length() - 1)));
		if (s.contains(".") || s.contains("e"))
			return new FloatLit(Float.parseFloat(s));
		if (s.startsWith("0x"))
			return new IntegerLit(Integer.parseInt(s.substring(2), 16));
		if (s.startsWith("0o"))
			return new IntegerLit(Integer.parseInt(s.substring(2), 8));
		if (s.startsWith("0b"))
			return new IntegerLit(Integer.parseInt(s.substring(2), 2));
		return new IntegerLit(Integer.parseInt(s));
	}

	private PythonNumericLiteral() {
	}
}
