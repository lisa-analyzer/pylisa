package it.unive.pylisa.analysis.namespaces;

/**
 * The binding status of a name inside a namespace (see {@link Frame}).
 */
public enum Binding {

	/**
	 * The status of names in unreachable code.
	 */
	BOTTOM,

	/**
	 * The name is definitely bound.
	 */
	BOUND,

	/**
	 * The name is definitely not bound.
	 */
	UNBOUND,

	/**
	 * The name might be bound or not.
	 */
	MAYBE;

	/**
	 * Yields whether the name might be bound.
	 *
	 * @return whether this is {@link #BOUND} or {@link #MAYBE}
	 */
	public boolean mightBeBound() {
		return this == BOUND || this == MAYBE;
	}

	/**
	 * Yields whether the name might be unbound.
	 *
	 * @return whether this is {@link #UNBOUND} or {@link #MAYBE}
	 */
	public boolean mightBeUnbound() {
		return this == UNBOUND || this == MAYBE;
	}
}
