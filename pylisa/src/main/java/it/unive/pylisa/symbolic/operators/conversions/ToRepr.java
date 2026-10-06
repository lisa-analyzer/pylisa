package it.unive.pylisa.symbolic.operators.conversions;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code repr(x)}.
 */
public class ToRepr implements UnaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final ToRepr INSTANCE = new ToRepr();

	private ToRepr() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(StringType.INSTANCE);
	}

	@Override
	public String toString() {
		return "repr";
	}
}
