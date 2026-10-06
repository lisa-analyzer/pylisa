package it.unive.pylisa.symbolic.operators.strings;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's searches of a substring within the {@code start}/{@code end} bounds
 * of a string: {@code s.find(sub, start, end)} and its variants. The operands
 * are the string, the substring and a slice holding the bounds (whose step is
 * ignored).
 */
public class StrSearch implements TernaryOperator {

	/**
	 * The kind of search.
	 */
	public enum Kind {
		/**
		 * {@code str.find}: the lowest index of the substring, or -1.
		 */
		FIND,
		/**
		 * {@code str.rfind}: the highest index of the substring, or -1.
		 */
		RFIND,
		/**
		 * {@code str.count}: the number of non-overlapping occurrences.
		 */
		COUNT,
		/**
		 * {@code str.startswith}.
		 */
		STARTSWITH,
		/**
		 * {@code str.endswith}.
		 */
		ENDSWITH
	}

	public static final StrSearch FIND = new StrSearch(Kind.FIND);
	public static final StrSearch RFIND = new StrSearch(Kind.RFIND);
	public static final StrSearch COUNT = new StrSearch(Kind.COUNT);
	public static final StrSearch STARTSWITH = new StrSearch(Kind.STARTSWITH);
	public static final StrSearch ENDSWITH = new StrSearch(Kind.ENDSWITH);

	private final Kind kind;

	private StrSearch(
			Kind kind) {
		this.kind = kind;
	}

	/**
	 * Yields the kind of search.
	 *
	 * @return the kind
	 */
	public Kind getKind() {
		return kind;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> middle,
			Set<Type> right) {
		return Collections.singleton(
				kind == Kind.STARTSWITH || kind == Kind.ENDSWITH ? BoolType.INSTANCE : Int32Type.INSTANCE);
	}

	@Override
	public String toString() {
		return "str." + kind.name().toLowerCase();
	}
}
