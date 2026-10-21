package it.unive.pylisa.symbolic.operators.compare;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.ComparisonOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * The negation of {@link IsInstance}, as in
 * {@code not isinstance(left, right)}.
 */
public class NotIsInstance implements ComparisonOperator, BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final NotIsInstance INSTANCE = new NotIsInstance();

	private NotIsInstance() {
	}

	@Override
	public ComparisonOperator opposite() {
		return IsInstance.INSTANCE;
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
		return "not isinstance";
	}
}
