package it.unive.pylisa.symbolic.operators.conversions;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * The condition "{@code int(x, base)}" (or "{@code float(x)}", ignoring the
 * right operand) "raises {@code ValueError}": e.g. {@code int("1.5")} or
 * {@code float("x")}. It is meant to be checked through
 * {@code Analysis#satisfies}: domains that can evaluate the conversion can tell
 * whether it raises, while all others answer that it might.
 */
public class ConversionRaises implements BinaryOperator {

	/**
	 * The condition for {@code int(x, base)}.
	 */
	public static final ConversionRaises INT = new ConversionRaises(true);

	/**
	 * The condition for {@code float(x)}.
	 */
	public static final ConversionRaises FLOAT = new ConversionRaises(false);

	private final boolean toInt;

	private ConversionRaises(
			boolean toInt) {
		this.toInt = toInt;
	}

	/**
	 * Whether this is the condition for {@code int(x, base)}.
	 *
	 * @return {@code true} for {@code int}, {@code false} for {@code float}
	 */
	public boolean isInt() {
		return toInt;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return Collections.singleton(BoolType.INSTANCE);
	}

	@Override
	public String toString() {
		return (toInt ? "int" : "float") + "%raises[ValueError]";
	}
}
