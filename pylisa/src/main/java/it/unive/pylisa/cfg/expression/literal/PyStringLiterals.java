package it.unive.pylisa.cfg.expression.literal;

/**
 * Decoding of Python string literals, as they appear in the source code, into
 * their values: the prefix ({@code r}, {@code u}, {@code b} and their
 * combinations, in any case) and the quotes are removed, and the escape
 * sequences are decoded unless the literal is raw. Bytes literals are decoded
 * as strings, since {@code bytes} is not modeled.
 */
public final class PyStringLiterals {

	private PyStringLiterals() {
	}

	/**
	 * The quotes delimiting a literal.
	 *
	 * @param literal the literal, as it appears in the source code
	 *
	 * @return the quotes ({@code '}, {@code "}, {@code '''} or {@code """})
	 */
	public static String quotes(
			String literal) {
		String body = literal.substring(prefixLength(literal));
		if (body.startsWith("'''") || body.startsWith("\"\"\""))
			return body.substring(0, 3);
		return body.substring(0, 1);
	}

	/**
	 * The value of a literal.
	 *
	 * @param literal the literal, as it appears in the source code
	 *
	 * @return its value
	 */
	public static String decode(
			String literal) {
		int prefix = prefixLength(literal);
		String prefixes = literal.substring(0, prefix).toLowerCase();
		String quotes = quotes(literal);
		String body = literal.substring(prefix + quotes.length(), literal.length() - quotes.length());
		return prefixes.contains("r") ? body : unescape(body, prefixes.contains("b"));
	}

	private static int prefixLength(
			String literal) {
		int i = 0;
		while (i < literal.length() && literal.charAt(i) != '\'' && literal.charAt(i) != '"')
			i++;
		return i;
	}

	private static String unescape(
			String body,
			boolean bytes) {
		StringBuilder sb = new StringBuilder();
		int n = body.length();
		for (int i = 0; i < n; i++) {
			char c = body.charAt(i);
			if (c != '\\' || i + 1 >= n) {
				sb.append(c);
				continue;
			}

			char e = body.charAt(++i);
			switch (e) {
			case '\n':
				// line continuation
				break;
			case '\r':
				// line continuation, with \r\n line endings
				if (i + 1 < n && body.charAt(i + 1) == '\n')
					i++;
				break;
			case '\\':
			case '\'':
			case '"':
				sb.append(e);
				break;
			case 'a':
				sb.append('\u0007');
				break;
			case 'b':
				sb.append('\b');
				break;
			case 'f':
				sb.append('\f');
				break;
			case 'n':
				sb.append('\n');
				break;
			case 'r':
				sb.append('\r');
				break;
			case 't':
				sb.append('\t');
				break;
			case 'v':
				sb.append('\u000b');
				break;
			case 'x':
				i = hex(body, i, 2, sb);
				break;
			case 'u':
				if (bytes)
					sb.append('\\').append(e);
				else
					i = hex(body, i, 4, sb);
				break;
			case 'U':
				if (bytes)
					sb.append('\\').append(e);
				else
					i = hex(body, i, 8, sb);
				break;
			case 'N': {
				if (bytes) {
					sb.append('\\').append(e);
					break;
				}
				int close = body.indexOf('}', i);
				if (i + 1 < n && body.charAt(i + 1) == '{' && close > 0) {
					try {
						sb.appendCodePoint(Character.codePointOf(body.substring(i + 2, close)));
						i = close;
						break;
					} catch (IllegalArgumentException ex) {
						// unknown name: a syntax error in python
					}
				}
				sb.append('\\').append(e);
				break;
			}
			default:
				if (e >= '0' && e <= '7') {
					// up to three octal digits
					int end = i;
					while (end < n && end < i + 3 && body.charAt(end) >= '0' && body.charAt(end) <= '7')
						end++;
					sb.appendCodePoint(Integer.parseInt(body.substring(i, end), 8));
					i = end - 1;
				} else
					// unknown escapes are kept as they are
					sb.append('\\').append(e);
			}
		}
		return sb.toString();
	}

	// decodes the hex digits following position i, returning the position of
	// the last one (or keeps the escape as it is, if they are not there)
	private static int hex(
			String body,
			int i,
			int digits,
			StringBuilder sb) {
		if (i + digits < body.length()) {
			String hex = body.substring(i + 1, i + 1 + digits);
			if (hex.chars().allMatch(ch -> Character.digit(ch, 16) >= 0)) {
				sb.appendCodePoint(Integer.parseInt(hex, 16));
				return i + digits;
			}
		}
		sb.append('\\').append(body.charAt(i));
		return i;
	}
}
