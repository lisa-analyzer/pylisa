package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code not x} on a value of any type: {@code True} if {@code x} is
 * false (e.g. {@code 0}, {@code ""} or {@code None}), {@code False} otherwise.
 * Unlike the logical negation of LiSA, it is not restricted to booleans.
 */
public class LogicalNot implements UnaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final LogicalNot INSTANCE = new LogicalNot();

	private LogicalNot() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(types.getBooleanType());
	}

	@Override
	public String toString() {
		return "not";
	}
}
