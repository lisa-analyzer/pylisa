package it.unive.pylisa.analysis.constants;

import java.math.BigInteger;

/**
 * Python's conversions of strings to numbers ({@code int(s, base)} and
 * {@code float(s)}), following CPython: surrounding whitespace is ignored,
 * single underscores are allowed between digits, and Unicode decimal digits are
 * accepted.
 */
public final class PyNumbers {

	private PyNumbers() {
	}

	/**
	 * Python's {@code int(s, base)}.
	 *
	 * @param s    the string
	 * @param base the base: 0 (deduced from the prefix) or between 2 and 36
	 *
	 * @return the value, or {@code null} if the conversion raises
	 *             {@code ValueError}
	 */
	public static BigInteger parseInt(
			String s,
			int base) {
		if (base != 0 && (base < 2 || base > 36))
			return null;
		String t = PyStrings.strip(s, null, true, true);
		int i = 0, n = t.length();
		boolean negative = false;
		if (i < n && (t.charAt(i) == '+' || t.charAt(i) == '-'))
			negative = t.charAt(i++) == '-';

		// the prefix matching the base, if any (or deciding it, for base 0)
		boolean prefixed = false;
		if (i + 1 < n && t.charAt(i) == '0') {
			char p = Character.toLowerCase(t.charAt(i + 1));
			int prefixBase = p == 'x' ? 16 : p == 'o' ? 8 : p == 'b' ? 2 : -1;
			if (prefixBase > 0 && (base == 0 || base == prefixBase)) {
				base = prefixBase;
				i += 2;
				prefixed = true;
			}
		}
		boolean decimalBase0 = base == 0;
		if (decimalBase0)
			base = 10;

		StringBuilder digits = new StringBuilder();
		// an underscore is allowed right after the prefix
		boolean underscoreOk = prefixed;
		for (; i < n; i++) {
			char c = t.charAt(i);
			if (c == '_') {
				if (!underscoreOk)
					return null;
				underscoreOk = false;
				continue;
			}
			int d = digit(c, base);
			if (d < 0)
				return null;
			digits.append(Character.forDigit(d, base));
			underscoreOk = true;
		}
		if (digits.length() == 0 || t.endsWith("_"))
			return null;
		// with base 0, decimal numbers cannot have leading zeros (except 0)
		if (decimalBase0 && digits.charAt(0) == '0' && digits.chars().anyMatch(c -> c != '0'))
			return null;
		BigInteger v = new BigInteger(digits.toString(), base);
		return negative ? v.negate() : v;
	}

	private static int digit(
			char c,
			int base) {
		int d;
		if (c < 128)
			d = Character.digit(c, base);
		else if (Character.getType(c) == Character.DECIMAL_DIGIT_NUMBER)
			// unicode decimal digits are accepted, other letters are not
			d = Character.digit(c, 10);
		else
			d = -1;
		return d < base ? d : -1;
	}

	/**
	 * Python's {@code float(s)}.
	 *
	 * @param s the string
	 *
	 * @return the value, or {@code null} if the conversion raises
	 *             {@code ValueError}
	 */
	public static Double parseFloat(
			String s) {
		String t = PyStrings.strip(s, null, true, true);
		int i = 0, n = t.length();
		boolean negative = false;
		if (i < n && (t.charAt(i) == '+' || t.charAt(i) == '-'))
			negative = t.charAt(i++) == '-';
		String rest = t.substring(i).toLowerCase();
		if (rest.equals("inf") || rest.equals("infinity"))
			return negative ? Double.NEGATIVE_INFINITY : Double.POSITIVE_INFINITY;
		if (rest.equals("nan"))
			return Double.NaN;

		// digits [. digits] [e [sign] digits], with underscores only between
		// digits
		StringBuilder num = new StringBuilder(negative ? "-" : "");
		int[] pos = { i };
		int intDigits = digits(t, pos, num);
		if (intDigits < 0)
			return null;
		int fracDigits = 0;
		if (pos[0] < n && t.charAt(pos[0]) == '.') {
			num.append('.');
			pos[0]++;
			fracDigits = digits(t, pos, num);
			if (fracDigits < 0)
				return null;
		}
		if (intDigits == 0 && fracDigits == 0)
			return null;
		if (pos[0] < n && (t.charAt(pos[0]) == 'e' || t.charAt(pos[0]) == 'E')) {
			num.append('e');
			pos[0]++;
			if (pos[0] < n && (t.charAt(pos[0]) == '+' || t.charAt(pos[0]) == '-'))
				num.append(t.charAt(pos[0]++));
			if (digits(t, pos, num) <= 0)
				return null;
		}
		if (pos[0] != n)
			return null;
		return Double.parseDouble(num.toString());
	}

	// reads decimal digits (with single underscores between them) from
	// pos[0], returning how many were read, or -1 if underscores are misplaced
	private static int digits(
			String t,
			int[] pos,
			StringBuilder out) {
		int count = 0;
		boolean lastUnderscore = false;
		for (; pos[0] < t.length(); pos[0]++) {
			char c = t.charAt(pos[0]);
			if (c == '_') {
				if (count == 0 || lastUnderscore)
					return -1;
				lastUnderscore = true;
				continue;
			}
			int d = c < 128 ? Character.digit(c, 10)
					: Character.getType(c) == Character.DECIMAL_DIGIT_NUMBER ? Character.digit(c, 10) : -1;
			if (d < 0)
				break;
			out.append((char) ('0' + d));
			count++;
			lastUnderscore = false;
		}
		return lastUnderscore ? -1 : count;
	}
}
