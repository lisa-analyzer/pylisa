package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code &}, {@code |} and {@code ^} between two {@code bool}s, whose
 * result is a {@code bool} (unlike the same operators between integers).
 */
public class BoolBitwise implements BinaryOperator {

	public static final BoolBitwise AND = new BoolBitwise("&");
	public static final BoolBitwise OR = new BoolBitwise("|");
	public static final BoolBitwise XOR = new BoolBitwise("^");

	private final String symbol;

	private BoolBitwise(
			String symbol) {
		this.symbol = symbol;
	}

	/**
	 * Applies the operator.
	 *
	 * @param left  the left operand
	 * @param right the right operand
	 *
	 * @return the result
	 */
	public boolean apply(
			boolean left,
			boolean right) {
		return this == AND ? left & right : this == OR ? left | right : left ^ right;
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
		return "bool" + symbol;
	}
}
