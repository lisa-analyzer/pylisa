package it.unive.pylisa.symbolic.operators.strings;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code s.strip(chars)} (and {@code lstrip}/{@code rstrip}): the
 * right operand holds the characters to remove, or {@code None} for whitespace.
 */
public class StrStrip implements BinaryOperator {

	public static final StrStrip STRIP = new StrStrip(true, true, "strip");
	public static final StrStrip LSTRIP = new StrStrip(true, false, "lstrip");
	public static final StrStrip RSTRIP = new StrStrip(false, true, "rstrip");

	private final boolean left, right;

	private final String name;

	private StrStrip(
			boolean left,
			boolean right,
			String name) {
		this.left = left;
		this.right = right;
		this.name = name;
	}

	/**
	 * Whether characters are removed from the beginning of the string.
	 *
	 * @return whether characters are removed from the beginning
	 */
	public boolean stripsLeft() {
		return left;
	}

	/**
	 * Whether characters are removed from the end of the string.
	 *
	 * @return whether characters are removed from the end
	 */
	public boolean stripsRight() {
		return right;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return Collections.singleton(StringType.INSTANCE);
	}

	@Override
	public String toString() {
		return "str." + name;
	}
}
