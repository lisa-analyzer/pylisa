package it.unive.pylisa.symbolic.operators.conversions;

import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code int(x, base)}: the right operand is the base, or {@code None}
 * if omitted. The {@code ValueError} raised for invalid strings is not part of
 * this operator (see {@link ConversionRaises}).
 */
public class ToInt implements BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final ToInt INSTANCE = new ToInt();

	private ToInt() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return Collections.singleton(Int32Type.INSTANCE);
	}

	@Override
	public String toString() {
		return "int";
	}
}
