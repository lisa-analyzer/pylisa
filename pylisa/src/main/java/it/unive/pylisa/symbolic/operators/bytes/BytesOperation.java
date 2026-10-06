package it.unive.pylisa.symbolic.operators.bytes;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyBytesType;
import java.util.Collections;
import java.util.Set;

/**
 * Python's binary operations on {@code bytes} (the left operand). The
 * exceptions they might raise are not part of the operators.
 */
public class BytesOperation implements BinaryOperator {

	/**
	 * {@code a + b}.
	 */
	public static final BytesOperation CONCAT = new BytesOperation("+", PyBytesType.INSTANCE);

	/**
	 * {@code b * n}.
	 */
	public static final BytesOperation REPEAT = new BytesOperation("*", PyBytesType.INSTANCE);

	/**
	 * {@code b[i]}, an {@code int}.
	 */
	public static final BytesOperation GETITEM = new BytesOperation("[]", Int32Type.INSTANCE);

	/**
	 * {@code b[slice]}.
	 */
	public static final BytesOperation GETSLICE = new BytesOperation("[:]", PyBytesType.INSTANCE);

	/**
	 * {@code x in b}, where {@code x} is an {@code int} or {@code bytes} (the
	 * right operand).
	 */
	public static final BytesOperation CONTAINS = new BytesOperation("contains", BoolType.INSTANCE);

	private final String name;

	private final Type result;

	private BytesOperation(
			String name,
			Type result) {
		this.name = name;
		this.result = result;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> right) {
		return Collections.singleton(result);
	}

	@Override
	public String toString() {
		return "bytes" + name;
	}
}
