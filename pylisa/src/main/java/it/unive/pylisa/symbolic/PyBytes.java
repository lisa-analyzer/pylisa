package it.unive.pylisa.symbolic;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;

/**
 * An immutable Python {@code bytes} value. The bytes can also be seen as the
 * Latin-1 string with one character per byte, so that string algorithms can be
 * reused on them.
 */
public final class PyBytes {

	private final byte[] data;

	/**
	 * Builds a value.
	 *
	 * @param data the bytes (copied)
	 */
	public PyBytes(
			byte[] data) {
		this.data = data.clone();
	}

	/**
	 * Builds the value whose bytes are the characters of a Latin-1 string.
	 *
	 * @param latin1 the string, with characters in the range 0-255
	 *
	 * @return the value
	 */
	public static PyBytes fromLatin1(
			String latin1) {
		return new PyBytes(latin1.getBytes(StandardCharsets.ISO_8859_1));
	}

	/**
	 * Yields the Latin-1 string with one character per byte.
	 *
	 * @return the string
	 */
	public String toLatin1() {
		return new String(data, StandardCharsets.ISO_8859_1);
	}

	/**
	 * Yields the number of bytes.
	 *
	 * @return the length
	 */
	public int length() {
		return data.length;
	}

	/**
	 * Yields the byte at the given position, as an integer between 0 and 255.
	 *
	 * @param i the position
	 *
	 * @return the byte
	 */
	public int get(
			int i) {
		return data[i] & 0xff;
	}

	/**
	 * Yields a copy of the bytes.
	 *
	 * @return the bytes
	 */
	public byte[] toArray() {
		return data.clone();
	}

	@Override
	public boolean equals(
			Object o) {
		return o instanceof PyBytes && Arrays.equals(data, ((PyBytes) o).data);
	}

	@Override
	public int hashCode() {
		return Arrays.hashCode(data);
	}

	/**
	 * Python's {@code repr} of the value (e.g. {@code b'a\x00'}), which is also
	 * its {@code str}.
	 */
	@Override
	public String toString() {
		boolean single = false, dbl = false;
		for (byte b : data) {
			single |= b == '\'';
			dbl |= b == '"';
		}
		char quote = single && !dbl ? '"' : '\'';
		StringBuilder sb = new StringBuilder("b").append(quote);
		for (byte b : data) {
			int c = b & 0xff;
			if (c == quote || c == '\\')
				sb.append('\\').append((char) c);
			else if (c == '\t')
				sb.append("\\t");
			else if (c == '\n')
				sb.append("\\n");
			else if (c == '\r')
				sb.append("\\r");
			else if (c < 0x20 || c >= 0x7f)
				sb.append(String.format("\\x%02x", c));
			else
				sb.append((char) c);
		}
		return sb.append(quote).toString();
	}
}
