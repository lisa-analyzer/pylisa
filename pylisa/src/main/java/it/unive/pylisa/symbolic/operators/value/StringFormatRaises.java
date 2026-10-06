package it.unive.pylisa.symbolic.operators.value;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collections;
import java.util.Set;

/**
 * The condition "{@code format % args} raises the given exception", where
 * {@code %} is Python's printf-style string formatting. It is meant to be
 * checked through {@code Analysis#satisfies} before formatting: domains that
 * can evaluate the formatting (e.g. constant propagation) can then tell whether
 * the exception is raised, while all others answer that it might be.
 */
public class StringFormatRaises implements BinaryOperator {

	/**
	 * The formatting raises {@code TypeError} (e.g. too many or too few
	 * arguments, or an argument of the wrong type).
	 */
	public static final StringFormatRaises TYPE_ERROR = new StringFormatRaises(
			LibrarySpecificationProvider.TYPE_ERROR);

	/**
	 * The formatting raises {@code ValueError} (e.g. an unsupported conversion
	 * character, or an incomplete format).
	 */
	public static final StringFormatRaises VALUE_ERROR = new StringFormatRaises(
			LibrarySpecificationProvider.VALUE_ERROR);

	private final String exception;

	private StringFormatRaises(
			String exception) {
		this.exception = exception;
	}

	/**
	 * Yields the name of the exception this condition is about.
	 *
	 * @return the name of the exception
	 */
	public String getException() {
		return exception;
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
		return "%raises[" + exception + "]";
	}
}
