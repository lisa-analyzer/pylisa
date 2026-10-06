package it.unive.pylisa.symbolic.operators.strings;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Set;

/**
 * Groups two values into a pair, so that operations with more than three
 * operands can be expressed as ternary expressions (e.g.
 * {@link StrReplaceCount}). A pair is not a Python value.
 */
public class ArgPair implements BinaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final ArgPair INSTANCE = new ArgPair();

	private ArgPair() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return Collections.singleton(Untyped.INSTANCE);
	}

	@Override
	public String toString() {
		return "pair";
	}
}
