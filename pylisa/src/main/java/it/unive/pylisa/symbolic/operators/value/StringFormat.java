package it.unive.pylisa.symbolic.operators.value;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's printf-style string formatting ({@code format % args}).
 */
public class StringFormat implements BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final StringFormat INSTANCE = new StringFormat();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected StringFormat() {
	}

	@Override
	public String toString() {
		return "%";
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		if (left.stream().noneMatch(Type::isStringType) && right.stream().noneMatch(Type::isStringType))
			return Collections.emptySet();
		return Collections.singleton(StringType.INSTANCE);
	}
}