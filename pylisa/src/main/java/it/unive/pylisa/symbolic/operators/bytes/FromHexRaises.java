package it.unive.pylisa.symbolic.operators.bytes;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * The condition "{@code bytes.fromhex(s)} raises {@code ValueError}" (i.e.,
 * {@code s} is not made of pairs of hexadecimal digits separated by
 * whitespace). It is meant to be checked through {@code Analysis#satisfies}.
 */
public class FromHexRaises implements UnaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final FromHexRaises INSTANCE = new FromHexRaises();

	private FromHexRaises() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(BoolType.INSTANCE);
	}

	@Override
	public String toString() {
		return "fromhex%raises[ValueError]";
	}
}
