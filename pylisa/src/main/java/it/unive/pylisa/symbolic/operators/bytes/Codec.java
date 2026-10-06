package it.unive.pylisa.symbolic.operators.bytes;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyBytesType;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code s.encode(encoding, errors)} and
 * {@code b.decode(encoding, errors)}: the operands are the value, the encoding
 * and the error handler ({@code None} for their defaults, {@code "utf-8"} and
 * {@code "strict"}). The exceptions they might raise are not part of the
 * operators (see {@link CodecRaises}).
 */
public class Codec implements TernaryOperator {

	/**
	 * {@code str.encode}.
	 */
	public static final Codec ENCODE = new Codec(true);

	/**
	 * {@code bytes.decode}.
	 */
	public static final Codec DECODE = new Codec(false);

	private final boolean encode;

	private Codec(
			boolean encode) {
		this.encode = encode;
	}

	/**
	 * Whether this is {@code str.encode}.
	 *
	 * @return {@code true} for encode, {@code false} for decode
	 */
	public boolean isEncode() {
		return encode;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> middle,
			Set<Type> right) {
		return Collections.singleton(encode ? PyBytesType.INSTANCE : StringType.INSTANCE);
	}

	@Override
	public String toString() {
		return encode ? "encode" : "decode";
	}
}
