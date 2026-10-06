package it.unive.pylisa.symbolic.operators.bytes;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyBytesType;
import java.util.Collections;
import java.util.Set;

/**
 * Python's unary operations producing or consuming {@code bytes}. The
 * exceptions they might raise are not part of the operators.
 */
public class BytesUnary implements UnaryOperator {

	/**
	 * {@code b.hex()}, a {@code str}.
	 */
	public static final BytesUnary HEX = new BytesUnary("hex", StringType.INSTANCE);

	/**
	 * {@code b.upper()}, changing only ASCII letters.
	 */
	public static final BytesUnary UPPER = new BytesUnary("upper", PyBytesType.INSTANCE);

	/**
	 * {@code b.lower()}, changing only ASCII letters.
	 */
	public static final BytesUnary LOWER = new BytesUnary("lower", PyBytesType.INSTANCE);

	/**
	 * {@code bytes.fromhex(s)}, for a {@code str} {@code s}.
	 */
	public static final BytesUnary FROMHEX = new BytesUnary("fromhex", PyBytesType.INSTANCE);

	/**
	 * {@code bytes(n)}: {@code n} zero bytes, for a non-negative {@code int}
	 * {@code n}.
	 */
	public static final BytesUnary ZEROS = new BytesUnary("zeros", PyBytesType.INSTANCE);

	private final String name;

	private final Type result;

	private BytesUnary(
			String name,
			Type result) {
		this.name = name;
		this.result = result;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> argument) {
		return Collections.singleton(result);
	}

	@Override
	public String toString() {
		return "bytes." + name;
	}
}
