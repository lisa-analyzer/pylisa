package it.unive.pylisa.analysis.constants;

import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.PyBytes;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.math.MathContext;
import java.math.RoundingMode;

/**
 * Python's printf-style string formatting ({@code format % arg}) between
 * constants, following CPython's {@code PyUnicode_Format}. Only a single scalar
 * argument ({@code int}, {@code float}, {@code str} or {@code bool}) is
 * supported: a tuple or a mapping as argument would need heap information, so
 * the outcome is undecided for any other kind of argument.
 * <p>
 * Python floats are double-precision, while they are tracked as Java
 * {@link Float}s: a float is converted back to the double with the same
 * shortest decimal representation (i.e., the {@code 0.1f} constant is formatted
 * as Python's {@code 0.1}).
 */
public final class PyPercentFormat {

	/**
	 * The outcome of a formatting.
	 */
	public static final class Result {

		/**
		 * Whether it is known if the formatting raises or not.
		 */
		public final boolean decided;

		/**
		 * The name of the exception raised, if any.
		 */
		public final String exception;

		/**
		 * The formatted string, or {@code null} if it raises or if it cannot be
		 * computed precisely.
		 */
		public final String value;

		private Result(
				boolean decided,
				String exception,
				String value) {
			this.decided = decided;
			this.exception = exception;
			this.value = value;
		}

		private static final Result UNDECIDED = new Result(false, null, null);

		private static Result raise(
				String exception) {
			return new Result(true, exception, null);
		}

		private static Result value(
				String value) {
			return new Result(true, null, value);
		}
	}

	private PyPercentFormat() {
	}

	/**
	 * Computes {@code format % arg}.
	 *
	 * @param format the format string
	 * @param arg    the argument (an {@link Integer}, {@link Long},
	 *                   {@link Float}, {@link Double}, {@link String} or
	 *                   {@link Boolean})
	 *
	 * @return the outcome of the formatting
	 */
	public static Result format(
			String format,
			Object arg) {
		if (!(arg instanceof Integer || arg instanceof Long || arg instanceof Float || arg instanceof Double
				|| arg instanceof String || arg instanceof Boolean || arg instanceof PyBytes))
			return Result.UNDECIDED;

		Object[] args = { arg };
		int next = 0;
		boolean known = true;
		StringBuilder out = new StringBuilder();
		int n = format.length();
		int i = 0;
		while (i < n) {
			char c = format.charAt(i++);
			if (c != '%') {
				out.append(c);
				continue;
			}

			int specStart = i;
			if (i >= n)
				return Result.raise(LibrarySpecificationProvider.VALUE_ERROR); // incomplete
																				// format
			if (format.charAt(i) == '(')
				// a scalar is never a mapping
				return Result.raise(LibrarySpecificationProvider.TYPE_ERROR);

			Spec spec = new Spec();
			for (; i < n && "-+ #0".indexOf(format.charAt(i)) >= 0; i++)
				spec.flag(format.charAt(i));

			if (i < n && format.charAt(i) == '*') {
				i++;
				if (next >= args.length)
					return Result.raise(LibrarySpecificationProvider.TYPE_ERROR);
				BigInteger w = asInt(args[next++]);
				if (w == null)
					return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // *
																					// wants
																					// int
				spec.width = w.intValue();
				if (spec.width < 0) {
					spec.left = true;
					spec.width = -spec.width;
				}
			} else
				for (; i < n && Character.isDigit(format.charAt(i)); i++)
					spec.width = Math.max(spec.width, 0) * 10 + (format.charAt(i) - '0');

			if (i < n && format.charAt(i) == '.') {
				i++;
				spec.precision = 0;
				if (i < n && format.charAt(i) == '*') {
					i++;
					if (next >= args.length)
						return Result.raise(LibrarySpecificationProvider.TYPE_ERROR);
					BigInteger p = asInt(args[next++]);
					if (p == null)
						return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // *
																						// wants
																						// int
					spec.precision = Math.max(p.intValue(), 0);
				} else
					for (; i < n && Character.isDigit(format.charAt(i)); i++)
						spec.precision = spec.precision * 10 + (format.charAt(i) - '0');
			}

			if (i < n && "hlL".indexOf(format.charAt(i)) >= 0)
				i++;
			if (i >= n)
				return Result.raise(LibrarySpecificationProvider.VALUE_ERROR); // incomplete
																				// format

			char conv = format.charAt(i++);
			if (conv == '%' && i - 1 == specStart) {
				// %% does not consume arguments
				out.append('%');
				continue;
			}

			// the argument is fetched before checking the conversion
			if (next >= args.length)
				return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // not
																				// enough
																				// arguments
			Object a = args[next++];

			String text;
			switch (conv) {
			case 's':
			case 'r':
			case 'a':
				text = conv == 's' ? str(a) : repr(a, conv == 'a');
				if (text != null && spec.precision >= 0 && spec.precision < text.length())
					text = text.substring(0, spec.precision);
				text = text == null ? null : pad(spec, "", text, false);
				break;
			case 'd':
			case 'i':
			case 'u':
				if (!(a instanceof Number || a instanceof Boolean))
					return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // a
																					// real
																					// number
																					// is
																					// required
				BigInteger dv = asInt(a);
				if (dv == null)
					dv = asDouble(a) == null || !Double.isFinite(asDouble(a)) ? null
							: new BigDecimal(asDouble(a)).toBigInteger();
				text = dv == null ? null : formatInteger(spec, dv, 10, false);
				break;
			case 'o':
			case 'x':
			case 'X':
				BigInteger iv = asInt(a);
				if (iv == null)
					return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // an
																					// integer
																					// is
																					// required
				text = formatInteger(spec, iv, conv == 'o' ? 8 : 16, conv == 'X');
				break;
			case 'e':
			case 'E':
			case 'f':
			case 'F':
			case 'g':
			case 'G':
				if (!(a instanceof Number || a instanceof Boolean))
					return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // must
																					// be
																					// real
																					// number
				text = formatFloat(spec, asDouble(a), conv);
				break;
			case 'c':
				if (a instanceof String) {
					if (((String) a).codePointCount(0, ((String) a).length()) != 1)
						return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // requires
																						// int
																						// or
																						// char
					text = pad(spec, "", (String) a, false);
				} else {
					BigInteger cv = asInt(a);
					if (cv == null)
						return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // requires
																						// int
																						// or
																						// char
					if (cv.signum() < 0 || cv.compareTo(BigInteger.valueOf(0x110000)) >= 0)
						// OverflowError, which is not modeled
						return Result.UNDECIDED;
					text = pad(spec, "", new String(Character.toChars(cv.intValue())), false);
				}
				break;
			default:
				return Result.raise(LibrarySpecificationProvider.VALUE_ERROR); // unsupported
																				// format
																				// character
			}

			if (text == null)
				known = false;
			else
				out.append(text);
		}

		if (next < args.length)
			return Result.raise(LibrarySpecificationProvider.TYPE_ERROR); // not
																			// all
																			// arguments
																			// converted

		return Result.value(known ? out.toString() : null);
	}

	private static final class Spec {
		boolean left, plus, space, alt, zero;
		int width = -1;
		int precision = -1;

		void flag(
				char c) {
			switch (c) {
			case '-':
				left = true;
				break;
			case '+':
				plus = true;
				break;
			case ' ':
				space = true;
				break;
			case '#':
				alt = true;
				break;
			case '0':
				zero = true;
				break;
			default:
				break;
			}
		}

		String sign(
				boolean negative) {
			return negative ? "-" : plus ? "+" : space ? " " : "";
		}
	}

	private static BigInteger asInt(
			Object a) {
		if (a instanceof Boolean)
			return ((Boolean) a) ? BigInteger.ONE : BigInteger.ZERO;
		if (a instanceof Integer || a instanceof Long)
			return BigInteger.valueOf(((Number) a).longValue());
		return null;
	}

	private static Double asDouble(
			Object a) {
		if (a instanceof Boolean)
			return ((Boolean) a) ? 1.0 : 0.0;
		if (a instanceof Float)
			// the double whose shortest representation is the one of the float
			// (zeros are returned as they are, since BigDecimal loses the sign
			// of -0.0)
			return Float.isFinite((Float) a) && (Float) a != 0 ? shortest((Float) a).doubleValue()
					: ((Float) a).doubleValue();
		if (a instanceof Number)
			return ((Number) a).doubleValue();
		return null;
	}

	/**
	 * The shortest decimal that rounds to the given float
	 * ({@link Float#toString} does not always yield it before JDK 19, e.g.
	 * {@code 1.00000003E16} for {@code 1e16f}).
	 */
	private static BigDecimal shortest(
			float f) {
		BigDecimal exact = new BigDecimal(f);
		for (int digits = 1; digits < 9; digits++) {
			BigDecimal candidate = exact.round(new MathContext(digits, RoundingMode.HALF_EVEN));
			if (candidate.floatValue() == f)
				return candidate;
		}
		return exact.round(new MathContext(9, RoundingMode.HALF_EVEN));
	}

	/**
	 * Pads {@code sign + prefix + body} to the width of the given spec;
	 * {@code numeric} enables zero padding, which goes after the sign and the
	 * prefix.
	 */
	private static String pad(
			Spec spec,
			String signAndPrefix,
			String body,
			boolean numeric) {
		int len = signAndPrefix.length() + body.length();
		if (spec.width <= len)
			return signAndPrefix + body;
		String fill = " ".repeat(spec.width - len);
		if (spec.left)
			return signAndPrefix + body + fill;
		if (numeric && spec.zero)
			return signAndPrefix + "0".repeat(spec.width - len) + body;
		return fill + signAndPrefix + body;
	}

	private static String formatInteger(
			Spec spec,
			BigInteger value,
			int radix,
			boolean upper) {
		String digits = value.abs().toString(radix);
		if (upper)
			digits = digits.toUpperCase();
		if (spec.precision > digits.length())
			digits = "0".repeat(spec.precision - digits.length()) + digits;
		String prefix = "";
		if (spec.alt && radix == 8)
			prefix = "0o";
		else if (spec.alt && radix == 16)
			prefix = upper ? "0X" : "0x";
		return pad(spec, spec.sign(value.signum() < 0) + prefix, digits, true);
	}

	private static String formatFloat(
			Spec spec,
			double value,
			char conv) {
		if (!Double.isFinite(value))
			return null;
		boolean negative = value < 0 || (value == 0 && 1 / value < 0);
		double abs = Math.abs(value);
		int p = spec.precision < 0 ? 6 : spec.precision;
		boolean upper = Character.isUpperCase(conv);

		String body;
		switch (Character.toLowerCase(conv)) {
		case 'f':
			body = fixed(abs, p);
			if (spec.alt && p == 0)
				body += ".";
			break;
		case 'e':
			body = scientific(abs, p, spec.alt);
			break;
		case 'g':
		default:
			if (p == 0)
				p = 1;
			int exp = abs == 0 ? 0 : exponent(abs, p);
			if (-4 <= exp && exp < p) {
				body = fixed(abs, p - 1 - exp);
				if (!spec.alt)
					body = stripZeros(body);
				else if (!body.contains("."))
					body += ".";
			} else {
				body = scientific(abs, p - 1, spec.alt);
				if (!spec.alt) {
					int e = body.indexOf('e');
					body = stripZeros(body.substring(0, e)) + body.substring(e);
				}
			}
			break;
		}
		if (upper)
			body = body.toUpperCase();
		return pad(spec, spec.sign(negative), body, true);
	}

	// correctly rounded (on the exact binary value) like C's printf
	private static String fixed(
			double abs,
			int precision) {
		return new BigDecimal(abs).setScale(precision, RoundingMode.HALF_EVEN).toPlainString();
	}

	// the decimal exponent of abs once rounded to the given significant digits
	private static int exponent(
			double abs,
			int digits) {
		BigDecimal rounded = new BigDecimal(abs).round(new MathContext(digits, RoundingMode.HALF_EVEN));
		return rounded.precision() - rounded.scale() - 1;
	}

	private static String scientific(
			double abs,
			int precision,
			boolean alt) {
		String digits;
		int exp;
		if (abs == 0) {
			digits = "0";
			exp = 0;
		} else {
			BigDecimal rounded = new BigDecimal(abs).round(new MathContext(precision + 1, RoundingMode.HALF_EVEN));
			digits = rounded.unscaledValue().toString();
			exp = digits.length() - 1 - rounded.scale();
		}
		if (digits.length() < precision + 1)
			digits = digits + "0".repeat(precision + 1 - digits.length());
		else
			digits = digits.substring(0, precision + 1);
		String mantissa = digits.substring(0, 1)
				+ (precision > 0 ? "." + digits.substring(1) : alt ? "." : "");
		return mantissa + exponentSuffix(exp);
	}

	private static String exponentSuffix(
			int exp) {
		String e = String.valueOf(Math.abs(exp));
		return "e" + (exp < 0 ? "-" : "+") + (e.length() < 2 ? "0" + e : e);
	}

	private static String stripZeros(
			String number) {
		if (!number.contains("."))
			return number;
		number = number.replaceAll("0+$", "");
		return number.endsWith(".") ? number.substring(0, number.length() - 1) : number;
	}

	/**
	 * Python's {@code str(a)}, or {@code null} if it cannot be computed.
	 */
	static String str(
			Object a) {
		if (a instanceof String)
			return (String) a;
		if (a instanceof PyBytes)
			// the str of bytes is their repr
			return a.toString();
		if (a instanceof Boolean)
			return ((Boolean) a) ? "True" : "False";
		if (a instanceof Integer || a instanceof Long)
			return a.toString();
		Double d = asDouble(a);
		if (d == null)
			return null;
		if (Double.isNaN(d))
			return "nan";
		if (Double.isInfinite(d))
			return d > 0 ? "inf" : "-inf";
		if (d == 0)
			return 1 / d < 0 ? "-0.0" : "0.0";

		// python's repr: the shortest representation, in positional notation
		// if 1e-4 <= |d| < 1e16, in scientific notation otherwise
		BigDecimal shortest = (a instanceof Float ? shortest((Float) a) : new BigDecimal(Double.toString(d)))
				.stripTrailingZeros();
		BigDecimal abs = shortest.abs();
		String sign = shortest.signum() < 0 ? "-" : "";
		if (abs.compareTo(new BigDecimal("1e-4")) >= 0 && abs.compareTo(new BigDecimal("1e16")) < 0) {
			String plain = abs.toPlainString();
			return sign + (plain.contains(".") ? plain : plain + ".0");
		}
		String digits = abs.unscaledValue().toString();
		int exp = digits.length() - 1 - abs.scale();
		return sign + digits.charAt(0) + (digits.length() > 1 ? "." + digits.substring(1) : "")
				+ exponentSuffix(exp);
	}

	/**
	 * Python's {@code repr(a)} (or {@code ascii(a)}), or {@code null} if it
	 * cannot be computed.
	 */
	static String repr(
			Object a,
			boolean ascii) {
		if (!(a instanceof String))
			// for numbers and bytes, repr is the same as str
			return str(a);
		String s = (String) a;
		char quote = s.indexOf('\'') >= 0 && s.indexOf('"') < 0 ? '"' : '\'';
		StringBuilder sb = new StringBuilder().append(quote);
		for (int k = 0; k < s.length(); k++) {
			char c = s.charAt(k);
			if (c == quote || c == '\\')
				sb.append('\\').append(c);
			else if (c == '\n')
				sb.append("\\n");
			else if (c == '\r')
				sb.append("\\r");
			else if (c == '\t')
				sb.append("\\t");
			else if (c < 0x20 || c == 0x7f)
				sb.append(String.format("\\x%02x", (int) c));
			else if (c < 0x7f)
				sb.append(c);
			else if (ascii && c < 0x100)
				sb.append(String.format("\\x%02x", (int) c));
			else
				// printability of non-ascii characters is not modeled
				return null;
		}
		return sb.append(quote).toString();
	}
}
