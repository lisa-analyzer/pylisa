package it.unive.pylisa.analysis;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * An abstract value read from an analysis state, in a form that tests can
 * compare against expectations independently of the value domain that computed
 * it.
 * <p>
 * A value is either one known concrete value ({@link Exact}), one of a finite
 * set of concrete values ({@link OneOf}), or unknown ({@link Top}).
 * </p>
 */
public sealed interface Val permits Val.Exact, Val.OneOf, Val.Top {

	/**
	 * A value known to be exactly {@code value}.
	 *
	 * @param value the concrete value (a {@link String}, a boxed number, a
	 *                  {@link Boolean}, ...)
	 */
	record Exact(Object value) implements Val {
	}

	/**
	 * A value known to be one of {@code values}, which holds at least two
	 * elements.
	 *
	 * @param values the possible concrete values
	 */
	record OneOf(Set<Object> values) implements Val {

		/**
		 * Builds the value, keeping an unmodifiable copy of the given values.
		 *
		 * @param values the possible concrete values
		 */
		public OneOf {
			values = Collections.unmodifiableSet(new LinkedHashSet<>(values));
		}
	}

	/**
	 * A value about which nothing is known.
	 */
	record Top() implements Val {
	}

	/**
	 * Yields the value known to be exactly {@code value}.
	 *
	 * @param value the concrete value
	 *
	 * @return the value
	 */
	static Val exact(
			Object value) {
		return new Exact(value);
	}

	/**
	 * Yields the unknown value.
	 *
	 * @return the value
	 */
	static Val top() {
		return new Top();
	}

	/**
	 * Yields the value that is one of the given concrete values: an
	 * {@link Exact} value when they are all the same, a {@link OneOf}
	 * otherwise.
	 *
	 * @param values the possible concrete values, at least one
	 *
	 * @return the value
	 */
	static Val oneOf(
			Collection<?> values) {
		Set<Object> distinct = new LinkedHashSet<>(values);
		if (distinct.isEmpty())
			throw new IllegalArgumentException("A value needs at least one possible concrete value");
		return distinct.size() == 1 ? new Exact(distinct.iterator().next()) : new OneOf(distinct);
	}

	/**
	 * Joins this value with another one: the result describes every concrete
	 * value described by either of them.
	 *
	 * @param other the other value
	 *
	 * @return the join
	 */
	default Val join(
			Val other) {
		if (this instanceof Top || other instanceof Top)
			return top();
		Set<Object> values = new LinkedHashSet<>(concreteValues());
		values.addAll(other.concreteValues());
		return oneOf(values);
	}

	/**
	 * Yields the concrete values this value describes.
	 *
	 * @return the concrete values
	 *
	 * @throws UnsupportedOperationException if this value is {@link Top}
	 */
	default Set<Object> concreteValues() {
		if (this instanceof Exact exact)
			return Set.of(exact.value());
		if (this instanceof OneOf oneOf)
			return oneOf.values();
		throw new UnsupportedOperationException("An unknown value has no finite set of concrete values");
	}
}
