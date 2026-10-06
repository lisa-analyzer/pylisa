package it.unive.pylisa.symbolic.operators.strings;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code s[i]} for a string {@code s} and an integer {@code i}: the
 * character at index {@code i}, where negative indexes count from the end. The
 * {@code IndexError} raised for an out-of-range index is not part of this
 * operator.
 */
public class StrGetItem implements BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final StrGetItem INSTANCE = new StrGetItem();

	private StrGetItem() {
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
		return "str[]";
	}
}
