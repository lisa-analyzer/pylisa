package it.unive.pylisa.symbolic.operators.strings;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code s[start:stop:step]} for a string {@code s}: its right operand
 * is the slice. Out-of-range bounds are clamped, as in Python. The
 * {@code ValueError} raised for a zero step is not part of this operator.
 */
public class StrGetSlice implements BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final StrGetSlice INSTANCE = new StrGetSlice();

	private StrGetSlice() {
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
		return "str[:]";
	}
}
