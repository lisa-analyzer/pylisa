package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's power ({@code a ** b}) when its result is a {@code float}: either
 * operand is a {@code float}, or the exponent is a negative {@code int}
 * ({@code 2 ** -1 == 0.5}). {@link Power} is instead the power between an
 * {@code int} and a non-negative {@code int}, whose result is an {@code int}.
 */
public class FloatPower extends Power {

	/**
	 * The singleton instance of this class.
	 */
	public static final FloatPower INSTANCE = new FloatPower();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected FloatPower() {
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
