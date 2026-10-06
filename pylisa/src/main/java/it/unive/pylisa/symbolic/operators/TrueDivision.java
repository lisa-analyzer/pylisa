package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingDiv;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's true division ({@code a / b}) between numbers. Unlike
 * {@link NumericNonOverflowingDiv}, whose result has the common type of its
 * operands, the result of a true division is always a {@code float}, even if
 * both operands are {@code int}s ({@code 6 / 3 == 2.0}).
 */
public class TrueDivision extends NumericNonOverflowingDiv {

	/**
	 * The singleton instance of this class.
	 */
	public static final TrueDivision INSTANCE = new TrueDivision();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected TrueDivision() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		if (left.stream().noneMatch(Type::isNumericType) || right.stream().noneMatch(Type::isNumericType))
			return Collections.emptySet();
		return Collections.singleton(Float32Type.INSTANCE);
	}
}
