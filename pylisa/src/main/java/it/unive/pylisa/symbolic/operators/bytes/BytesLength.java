package it.unive.pylisa.symbolic.operators.bytes;

import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code len(b)} for {@code bytes}.
 */
public class BytesLength implements UnaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final BytesLength INSTANCE = new BytesLength();

	private BytesLength() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(Int32Type.INSTANCE);
	}

	@Override
	public String toString() {
		return "len";
	}
}
