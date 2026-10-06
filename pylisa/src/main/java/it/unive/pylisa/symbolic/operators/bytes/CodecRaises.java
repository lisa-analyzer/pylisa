package it.unive.pylisa.symbolic.operators.bytes;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * The condition "the {@link Codec} on the same operands raises the given
 * exception". It is meant to be checked through {@code Analysis#satisfies}:
 * domains that can evaluate the codec can tell whether it raises, while all
 * others answer that it might.
 */
public class CodecRaises implements TernaryOperator {

	private final Codec codec;

	private final String exception;

	/**
	 * Builds the condition.
	 *
	 * @param codec     the codec
	 * @param exception the name of the exception
	 */
	public CodecRaises(
			Codec codec,
			String exception) {
		this.codec = codec;
		this.exception = exception;
	}

	/**
	 * Yields the codec.
	 *
	 * @return the codec
	 */
	public Codec getCodec() {
		return codec;
	}

	/**
	 * Yields the name of the exception.
	 *
	 * @return the exception
	 */
	public String getException() {
		return exception;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> middle,
			Set<Type> right) {
		return Collections.singleton(BoolType.INSTANCE);
	}

	@Override
	public boolean equals(
			Object o) {
		return o instanceof CodecRaises && ((CodecRaises) o).codec == codec
				&& ((CodecRaises) o).exception.equals(exception);
	}

	@Override
	public int hashCode() {
		return codec.hashCode() ^ exception.hashCode();
	}

	@Override
	public String toString() {
		return codec + "%raises[" + exception + "]";
	}
}
