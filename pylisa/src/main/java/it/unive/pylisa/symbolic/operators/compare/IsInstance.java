package it.unive.pylisa.symbolic.operators.compare;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.ComparisonOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code isinstance(left, right)} (see {@code IsInstanceBuiltin}),
 * when its result is not known: the branches of a condition refine the types of
 * {@code left} (e.g., in the true branch of {@code isinstance(x, str)},
 * {@code x} is a {@code str}).
 */
public class IsInstance implements ComparisonOperator, BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final IsInstance INSTANCE = new IsInstance();

	private IsInstance() {
	}

	@Override
	public ComparisonOperator opposite() {
		return NotIsInstance.INSTANCE;
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
		return "isinstance";
	}
}
