package it.unive.pylisa.analysis.constants;

import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.PyBytes;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.charset.CharacterCodingException;
import java.nio.charset.Charset;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.Set;

/**
 * Python's {@code bytes.decode(encoding, errors)} and
 * {@code str.encode(encoding, errors)} between constants. Only the UTF-8, ASCII
 * and Latin-1 codecs (under any of their Python names) and the {@code strict},
 * {@code ignore} and {@code replace} error handlers are modeled: the outcome is
 * undecided for any other codec or handler, rather than a possibly wrong
 * {@code LookupError}.
 */
public final class PyCodecs {

	private PyCodecs() {
	}

	/**
	 * The outcome of an encoding or decoding.
	 */
	public static final class Result {

		/**
		 * Whether it is known if the operation raises or not.
		 */
		public final boolean decided;

		/**
		 * The name of the exception raised, if any.
		 */
		public final String exception;

		/**
		 * The result ({@link String} or {@link PyBytes}), or {@code null} if it
		 * raises.
		 */
		public final Object value;

		private Result(
				boolean decided,
				String exception,
				Object value) {
			this.decided = decided;
			this.exception = exception;
			this.value = value;
		}

		private static final Result UNDECIDED = new Result(false, null, null);
	}

	/**
	 * The exception raised when decoding fails.
	 */
	public static final String DECODE_ERROR = LibrarySpecificationProvider.UNICODE_DECODE_ERROR;

	/**
	 * The exception raised when encoding fails.
	 */
	public static final String ENCODE_ERROR = LibrarySpecificationProvider.UNICODE_ENCODE_ERROR;

	private static final Map<String, Charset> CODECS = Map.ofEntries(
			Map.entry("utf_8", StandardCharsets.UTF_8),
			Map.entry("cp65001", StandardCharsets.UTF_8),
			Map.entry("u8", StandardCharsets.UTF_8),
			Map.entry("utf", StandardCharsets.UTF_8),
			Map.entry("utf8", StandardCharsets.UTF_8),
			Map.entry("utf8_ucs2", StandardCharsets.UTF_8),
			Map.entry("utf8_ucs4", StandardCharsets.UTF_8),
			Map.entry("ascii", StandardCharsets.US_ASCII),
			Map.entry("646", StandardCharsets.US_ASCII),
			Map.entry("ansi_x3.4_1968", StandardCharsets.US_ASCII),
			Map.entry("ansi_x3.4_1986", StandardCharsets.US_ASCII),
			Map.entry("ansi_x3_4_1968", StandardCharsets.US_ASCII),
			Map.entry("cp367", StandardCharsets.US_ASCII),
			Map.entry("csascii", StandardCharsets.US_ASCII),
			Map.entry("ibm367", StandardCharsets.US_ASCII),
			Map.entry("iso646_us", StandardCharsets.US_ASCII),
			Map.entry("iso_646.irv_1991", StandardCharsets.US_ASCII),
			Map.entry("iso_ir_6", StandardCharsets.US_ASCII),
			Map.entry("us", StandardCharsets.US_ASCII),
			Map.entry("us_ascii", StandardCharsets.US_ASCII),
			Map.entry("latin_1", StandardCharsets.ISO_8859_1),
			Map.entry("8859", StandardCharsets.ISO_8859_1),
			Map.entry("cp819", StandardCharsets.ISO_8859_1),
			Map.entry("csisolatin1", StandardCharsets.ISO_8859_1),
			Map.entry("ibm819", StandardCharsets.ISO_8859_1),
			Map.entry("iso8859", StandardCharsets.ISO_8859_1),
			Map.entry("iso8859_1", StandardCharsets.ISO_8859_1),
			Map.entry("iso_8859_1", StandardCharsets.ISO_8859_1),
			Map.entry("iso_8859_1_1987", StandardCharsets.ISO_8859_1),
			Map.entry("iso_ir_100", StandardCharsets.ISO_8859_1),
			Map.entry("l1", StandardCharsets.ISO_8859_1),
			Map.entry("latin", StandardCharsets.ISO_8859_1),
			Map.entry("latin1", StandardCharsets.ISO_8859_1));

	// the names of the modules implementing the codecs, that are not aliases
	private static final Set<String> MODULES = Set.of("utf_8", "ascii", "latin_1");

	private static final Set<String> HANDLERS = Set.of("strict", "ignore", "replace");

	/**
	 * The codec with the given Python name, or {@code null} if it is not one of
	 * the modeled ones.
	 *
	 * @param name the name of the encoding
	 *
	 * @return the codec, or {@code null}
	 */
	public static Charset codec(
			String name) {
		// encodings.normalize_encoding, after lowercasing
		StringBuilder sb = new StringBuilder();
		boolean punct = false;
		for (char c : name.toLowerCase().toCharArray()) {
			if (Character.isLetterOrDigit(c) || c == '.') {
				if (punct && sb.length() > 0)
					sb.append('_');
				if (c < 128)
					sb.append(c);
				punct = false;
			} else
				punct = true;
		}
		String norm = sb.toString();
		Charset cs = CODECS.get(norm);
		if (cs != null)
			return cs;
		// encodings.search_function retries aliases (not module names) with
		// dots replaced by underscores
		String undotted = norm.replace('.', '_');
		return MODULES.contains(undotted) ? null : CODECS.get(undotted);
	}

	/**
	 * Python's {@code data.decode(encoding, errors)}.
	 *
	 * @param data     the bytes
	 * @param encoding the encoding
	 * @param errors   the error handler
	 *
	 * @return the outcome
	 */
	public static Result decode(
			PyBytes data,
			String encoding,
			String errors) {
		Charset cs = codec(encoding);
		if (cs == null)
			return Result.UNDECIDED;
		if (cs.equals(StandardCharsets.UTF_8))
			return decodeUtf8(data, errors);
		try {
			String s = cs.newDecoder().onMalformedInput(CodingErrorAction.REPORT)
					.onUnmappableCharacter(CodingErrorAction.REPORT)
					.decode(ByteBuffer.wrap(data.toArray())).toString();
			return new Result(true, null, s);
		} catch (CharacterCodingException e) {
			// the handler is only looked up when an error occurs
			if (!HANDLERS.contains(errors))
				return Result.UNDECIDED;
			if (errors.equals("strict"))
				return new Result(true, DECODE_ERROR, null);
			CodingErrorAction action = errors.equals("ignore") ? CodingErrorAction.IGNORE
					: CodingErrorAction.REPLACE;
			try {
				String s = cs.newDecoder().onMalformedInput(action).onUnmappableCharacter(action)
						.replaceWith("�").decode(ByteBuffer.wrap(data.toArray())).toString();
				return new Result(true, null, s);
			} catch (CharacterCodingException ex) {
				return Result.UNDECIDED;
			}
		}
	}

	/**
	 * Decodes UTF-8 as CPython does: each maximal invalid subpart of a sequence
	 * is a single error (e.g. the encoded surrogate {@code ED A0 80} is three
	 * errors, since {@code ED} cannot be followed by {@code A0}), which the
	 * JDK's decoder does not follow.
	 */
	private static Result decodeUtf8(
			PyBytes data,
			String errors) {
		StringBuilder sb = new StringBuilder();
		int n = data.length();
		int i = 0;
		while (i < n) {
			int b0 = data.get(i);
			if (b0 < 0x80) {
				sb.append((char) b0);
				i++;
				continue;
			}

			// the number of continuation bytes, and the range of the first one
			int count, lo = 0x80, hi = 0xbf;
			if (b0 >= 0xc2 && b0 <= 0xdf)
				count = 1;
			else if (b0 >= 0xe0 && b0 <= 0xef) {
				count = 2;
				if (b0 == 0xe0)
					lo = 0xa0;
				else if (b0 == 0xed)
					hi = 0x9f;
			} else if (b0 >= 0xf0 && b0 <= 0xf4) {
				count = 3;
				if (b0 == 0xf0)
					lo = 0x90;
				else if (b0 == 0xf4)
					hi = 0x8f;
			} else
				count = -1;

			int cp = b0 & (count == 1 ? 0x1f : count == 2 ? 0x0f : 0x07);
			int j = i + 1;
			boolean valid = count > 0;
			for (int k = 0; valid && k < count; k++, j++) {
				int min = k == 0 ? lo : 0x80, max = k == 0 ? hi : 0xbf;
				if (j >= n || data.get(j) < min || data.get(j) > max)
					valid = false;
				else
					cp = (cp << 6) | (data.get(j) & 0x3f);
			}

			if (valid) {
				sb.appendCodePoint(cp);
				i = j;
				continue;
			}

			// the error covers the lead byte and the valid continuation bytes
			// before the first invalid one
			if (!HANDLERS.contains(errors))
				return Result.UNDECIDED;
			if (errors.equals("strict"))
				return new Result(true, DECODE_ERROR, null);
			if (errors.equals("replace"))
				sb.append('\ufffd');
			// the first invalid byte is examined again, as a lead byte
			i = count > 0 ? j - 1 : i + 1;
		}
		return new Result(true, null, sb.toString());
	}

	/**
	 * Python's {@code s.encode(encoding, errors)}.
	 *
	 * @param s        the string
	 * @param encoding the encoding
	 * @param errors   the error handler
	 *
	 * @return the outcome
	 */
	public static Result encode(
			String s,
			String encoding,
			String errors) {
		Charset cs = codec(encoding);
		if (cs == null)
			return Result.UNDECIDED;
		try {
			return new Result(true, null, bytes(cs.newEncoder().onMalformedInput(CodingErrorAction.REPORT)
					.onUnmappableCharacter(CodingErrorAction.REPORT).encode(CharBuffer.wrap(s))));
		} catch (CharacterCodingException e) {
			if (!HANDLERS.contains(errors))
				return Result.UNDECIDED;
			if (errors.equals("strict"))
				return new Result(true, ENCODE_ERROR, null);
			// with "replace", each code point that cannot be encoded becomes ?
			StringBuilder out = new StringBuilder();
			s.codePoints().forEach(cp -> {
				String c = new String(Character.toChars(cp));
				if (cs.newEncoder().canEncode(c))
					out.append(c);
				else if (errors.equals("replace"))
					out.append('?');
			});
			try {
				return new Result(true, null, bytes(cs.newEncoder().encode(CharBuffer.wrap(out))));
			} catch (CharacterCodingException ex) {
				return Result.UNDECIDED;
			}
		}
	}

	private static PyBytes bytes(
			ByteBuffer buffer) {
		byte[] b = new byte[buffer.remaining()];
		buffer.get(b);
		return new PyBytes(b);
	}

	/**
	 * Python's {@code data.hex()}.
	 *
	 * @param data the bytes
	 *
	 * @return the hexadecimal string
	 */
	public static String hex(
			PyBytes data) {
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < data.length(); i++)
			sb.append(String.format("%02x", data.get(i)));
		return sb.toString();
	}

	/**
	 * Python's {@code bytes.fromhex(s)}: pairs of hexadecimal digits,
	 * optionally separated by ASCII whitespace.
	 *
	 * @param s the string
	 *
	 * @return the bytes, or {@code null} if {@code ValueError} is raised
	 */
	public static PyBytes fromHex(
			String s) {
		StringBuilder out = new StringBuilder();
		int i = 0, n = s.length();
		while (true) {
			while (i < n && isAsciiSpace(s.charAt(i)))
				i++;
			if (i >= n)
				break;
			if (i + 1 >= n)
				return null;
			int hi = hexDigit(s.charAt(i)), lo = hexDigit(s.charAt(i + 1));
			if (hi < 0 || lo < 0)
				return null;
			out.append((char) (hi * 16 + lo));
			i += 2;
		}
		return PyBytes.fromLatin1(out.toString());
	}

	private static int hexDigit(
			char c) {
		return c < 128 ? Character.digit(c, 16) : -1;
	}

	/**
	 * Whether a character is ASCII whitespace, as for {@code bytes.isspace}.
	 *
	 * @param c the character
	 *
	 * @return whether it is whitespace
	 */
	public static boolean isAsciiSpace(
			int c) {
		return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == 0x0b || c == 0x0c;
	}

	/**
	 * The ASCII whitespace characters, that {@code bytes.strip()} removes.
	 */
	public static final String ASCII_WHITESPACE = " \t\n\r\u000b\u000c";

	/**
	 * Python's {@code data.upper()} or {@code data.lower()}: only ASCII letters
	 * are changed.
	 *
	 * @param data  the bytes
	 * @param upper whether to convert to upper case
	 *
	 * @return the converted bytes
	 */
	public static PyBytes asciiCase(
			PyBytes data,
			boolean upper) {
		StringBuilder sb = new StringBuilder();
		for (char c : data.toLatin1().toCharArray())
			sb.append(upper && c >= 'a' && c <= 'z' ? (char) (c - 32)
					: !upper && c >= 'A' && c <= 'Z' ? (char) (c + 32) : c);
		return PyBytes.fromLatin1(sb.toString());
	}
}
