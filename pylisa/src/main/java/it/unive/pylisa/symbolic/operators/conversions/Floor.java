package it.unive.pylisa.symbolic.operators.conversions;

import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code math.floor(x)} on numbers: the largest {@code int} not
 * greater than {@code x}. The {@code ValueError} and {@code OverflowError}
 * raised for {@code nan} and infinities are not part of this operator (see
 * {@code MathFloor}).
 */
public class Floor implements UnaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final Floor INSTANCE = new Floor();

	private Floor() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(Int32Type.INSTANCE);
	}

	@Override
	public String toString() {
		return "floor";
	}
}
