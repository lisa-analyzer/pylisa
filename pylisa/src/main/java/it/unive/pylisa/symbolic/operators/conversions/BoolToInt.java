package it.unive.pylisa.symbolic.operators.conversions;

import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * The {@code int} value of a {@code bool} ({@code 0} or {@code 1}): since
 * {@code bool} is a subclass of {@code int}, bools are converted with this
 * operator when they are used where an {@code int} is expected (e.g.
 * {@code True + 1}). Integers are left unchanged.
 */
public class BoolToInt implements UnaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final BoolToInt INSTANCE = new BoolToInt();

	private BoolToInt() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(Int32Type.INSTANCE);
	}

	@Override
	public String toString() {
		return "int";
	}
}
