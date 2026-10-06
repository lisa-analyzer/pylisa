package it.unive.pylisa.analysis.constants;

import java.util.Locale;
import java.util.Objects;

/**
 * Python's string operations between constants, following CPython. Strings are
 * sequences of code points, as in Python (and not of UTF-16 units, as in Java).
 * Bounds of slices and of searches are {@link Long}s, with {@code null}
 * standing for {@code None}.
 */
public final class PyStrings {

	private PyStrings() {
	}

	/**
	 * A Python slice {@code start:stop:step}, where {@code null} stands for an
	 * omitted bound (or {@code None}).
	 */
	public static final class Slice {

		/**
		 * The bounds and the step.
		 */
		public final Long start, stop, step;

		/**
		 * Builds the slice.
		 *
		 * @param start the start, or {@code null}
		 * @param stop  the stop, or {@code null}
		 * @param step  the step, or {@code null}
		 */
		public Slice(
				Long start,
				Long stop,
				Long step) {
			this.start = start;
			this.stop = stop;
			this.step = step;
		}

		@Override
		public boolean equals(
				Object o) {
			if (!(o instanceof Slice))
				return false;
			Slice s = (Slice) o;
			return Objects.equals(start, s.start) && Objects.equals(stop, s.stop) && Objects.equals(step, s.step);
		}

		@Override
		public int hashCode() {
			return Objects.hash(start, stop, step);
		}

		@Override
		public String toString() {
			return "slice(" + str(start) + ", " + str(stop) + ", " + str(step) + ")";
		}

		private static String str(
				Long l) {
			return l == null ? "None" : l.toString();
		}
	}

	/**
	 * A pair of values (see {@code ArgPair}).
	 */
	public static final class Pair {

		/**
		 * The values.
		 */
		public final Object first, second;

		/**
		 * Builds the pair.
		 *
		 * @param first  the first value
		 * @param second the second value
		 */
		public Pair(
				Object first,
				Object second) {
			this.first = first;
			this.second = second;
		}

		@Override
		public boolean equals(
				Object o) {
			return o instanceof Pair && Objects.equals(first, ((Pair) o).first)
					&& Objects.equals(second, ((Pair) o).second);
		}

		@Override
		public int hashCode() {
			return Objects.hash(first, second);
		}

		@Override
		public String toString() {
			return "(" + first + ", " + second + ")";
		}
	}

	private static int[] cps(
			String s) {
		return s.codePoints().toArray();
	}

	private static String str(
			int[] cps,
			int from,
			int to) {
		return new String(cps, from, Math.max(0, to - from));
	}

	/**
	 * The length of {@code s}, in code points.
	 *
	 * @param s the string
	 *
	 * @return the length
	 */
	public static int length(
			String s) {
		return s.codePointCount(0, s.length());
	}

	/**
	 * Python's {@code s[i]}.
	 *
	 * @param s the string
	 * @param i the index
	 *
	 * @return the character, or {@code null} if the index is out of range
	 *             ({@code IndexError})
	 */
	public static String getItem(
			String s,
			long i) {
		int[] cps = cps(s);
		if (i < 0)
			i += cps.length;
		if (i < 0 || i >= cps.length)
			return null;
		return str(cps, (int) i, (int) i + 1);
	}

	/**
	 * Python's {@code s[slice]}.
	 *
	 * @param s     the string
	 * @param slice the slice
	 *
	 * @return the substring, or {@code null} if the step is zero
	 *             ({@code ValueError})
	 */
	public static String getSlice(
			String s,
			Slice slice) {
		long step = slice.step == null ? 1 : slice.step;
		if (step == 0)
			return null;
		int[] cps = cps(s);
		long len = cps.length;

		// PySlice_Unpack and PySlice_AdjustIndices
		long start = slice.start == null ? (step < 0 ? Long.MAX_VALUE : 0) : slice.start;
		long stop = slice.stop == null ? (step < 0 ? Long.MIN_VALUE : Long.MAX_VALUE) : slice.stop;
		if (start < 0) {
			start += len;
			if (start < 0)
				start = step < 0 ? -1 : 0;
		} else if (start >= len)
			start = step < 0 ? len - 1 : len;
		if (stop < 0) {
			stop += len;
			if (stop < 0)
				stop = step < 0 ? -1 : 0;
		} else if (stop >= len)
			stop = step < 0 ? len - 1 : len;

		StringBuilder sb = new StringBuilder();
		if (step > 0)
			for (long k = start; k < stop; k += step)
				sb.appendCodePoint(cps[(int) k]);
		else
			for (long k = start; k > stop; k += step)
				sb.appendCodePoint(cps[(int) k]);
		return sb.toString();
	}

	/**
	 * Python's searches {@code s.find(sub, start, end)}, {@code s.rfind(...)},
	 * {@code s.count(...)}, {@code s.startswith(...)} and
	 * {@code s.endswith(...)}.
	 *
	 * @param kind   the kind of search: {@code find}, {@code rfind},
	 *                   {@code count}, {@code startswith} or {@code endswith}
	 * @param s      the string
	 * @param sub    the substring
	 * @param bounds the slice holding the bounds (its step is ignored)
	 *
	 * @return the result ({@link Integer} or {@link Boolean})
	 */
	public static Object search(
			String kind,
			String s,
			String sub,
			Slice bounds) {
		int[] cps = cps(s);
		int[] subCps = cps(sub);
		long len = cps.length;
		long n = subCps.length;

		// ADJUST_INDICES: end is clamped to len, start is not
		long start = bounds.start == null ? 0 : bounds.start;
		long end = bounds.stop == null ? Long.MAX_VALUE : bounds.stop;
		if (end > len)
			end = len;
		else if (end < 0) {
			end += len;
			if (end < 0)
				end = 0;
		}
		if (start < 0) {
			start += len;
			if (start < 0)
				start = 0;
		}

		switch (kind) {
		case "startswith":
		case "endswith":
			// tailmatch
			if (end - n < start)
				return false;
			long from = kind.equals("startswith") ? start : end - n;
			return regionMatches(cps, (int) from, subCps);
		case "count":
			if (end - start < n)
				return 0;
			if (n == 0)
				return (int) (end - start + 1);
			int count = 0;
			for (long k = start; k + n <= end;)
				if (regionMatches(cps, (int) k, subCps)) {
					count++;
					k += n;
				} else
					k++;
			return count;
		case "rfind":
			if (end - start < n)
				return -1;
			for (long k = end - n; k >= start; k--)
				if (regionMatches(cps, (int) k, subCps))
					return (int) k;
			return -1;
		case "find":
		default:
			if (end - start < n)
				return -1;
			for (long k = start; k + n <= end; k++)
				if (regionMatches(cps, (int) k, subCps))
					return (int) k;
			return -1;
		}
	}

	private static boolean regionMatches(
			int[] cps,
			int from,
			int[] sub) {
		for (int k = 0; k < sub.length; k++)
			if (cps[from + k] != sub[k])
				return false;
		return true;
	}

	/**
	 * Python's {@code s.replace(old, new, count)}.
	 *
	 * @param s     the string
	 * @param old   the substring to replace
	 * @param repl  the replacement
	 * @param count the maximum number of replacements, or a negative number for
	 *                  all of them
	 *
	 * @return the resulting string
	 */
	public static String replace(
			String s,
			String old,
			String repl,
			long count) {
		if (count < 0)
			count = Long.MAX_VALUE;
		int[] cps = cps(s);
		int[] oldCps = cps(old);
		StringBuilder sb = new StringBuilder();
		long done = 0;
		if (oldCps.length == 0) {
			// the replacement is inserted before each character, and at the
			// end
			for (int k = 0; k < cps.length; k++) {
				if (done < count) {
					sb.append(repl);
					done++;
				}
				sb.appendCodePoint(cps[k]);
			}
			if (done < count)
				sb.append(repl);
			return sb.toString();
		}

		int k = 0;
		while (k < cps.length)
			if (done < count && k + oldCps.length <= cps.length && regionMatches(cps, k, oldCps)) {
				sb.append(repl);
				done++;
				k += oldCps.length;
			} else
				sb.appendCodePoint(cps[k++]);
		return sb.toString();
	}

	/**
	 * Python's {@code s.strip(chars)}, {@code s.lstrip(chars)} and
	 * {@code s.rstrip(chars)}.
	 *
	 * @param s     the string
	 * @param chars the characters to remove, or {@code null} for whitespace
	 * @param left  whether to remove characters from the beginning
	 * @param right whether to remove characters from the end
	 *
	 * @return the resulting string
	 */
	public static String strip(
			String s,
			String chars,
			boolean left,
			boolean right) {
		int[] cps = cps(s);
		int[] set = chars == null ? null : cps(chars);
		int from = 0, to = cps.length;
		if (left)
			while (from < to && strips(cps[from], set))
				from++;
		if (right)
			while (to > from && strips(cps[to - 1], set))
				to--;
		return str(cps, from, to);
	}

	private static boolean strips(
			int cp,
			int[] set) {
		if (set == null)
			return isSpace(cp);
		for (int c : set)
			if (c == cp)
				return true;
		return false;
	}

	/**
	 * Python's {@code str.isspace} for a single code point.
	 *
	 * @param cp the code point
	 *
	 * @return whether it is whitespace
	 */
	public static boolean isSpace(
			int cp) {
		if (cp == ' ' || (cp >= 0x09 && cp <= 0x0d) || (cp >= 0x1c && cp <= 0x1f) || cp == 0x85)
			return true;
		int type = Character.getType(cp);
		return type == Character.SPACE_SEPARATOR || type == Character.LINE_SEPARATOR
				|| type == Character.PARAGRAPH_SEPARATOR;
	}

	/**
	 * Python's {@code s.upper()}.
	 *
	 * @param s the string
	 *
	 * @return the resulting string
	 */
	public static String upper(
			String s) {
		return s.toUpperCase(Locale.ROOT);
	}

	/**
	 * Python's {@code s.lower()}.
	 *
	 * @param s the string
	 *
	 * @return the resulting string
	 */
	public static String lower(
			String s) {
		return s.toLowerCase(Locale.ROOT);
	}
}
