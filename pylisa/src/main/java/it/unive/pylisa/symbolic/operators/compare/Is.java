package it.unive.pylisa.symbolic.operators.compare;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.ComparisonOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's identity test {@code left is right} (see {@code PyIs}), when its
 * result is not known: the branches of a condition refine the operands (e.g.,
 * in the branches of {@code x is None}, {@code x} is {@code None} or it is
 * not).
 */
public class Is implements ComparisonOperator, BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final Is INSTANCE = new Is();

	private Is() {
	}

	@Override
	public ComparisonOperator opposite() {
		return IsNot.INSTANCE;
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
		return "is";
	}
}
