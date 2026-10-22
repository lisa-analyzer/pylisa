package it.unive.pylisa.scoping;

/**
 * The kind of a name inside a {@link PyScope}, determining where the name is
 * looked up and bound. The kinds follow the ones of CPython's symbol tables.
 */
public enum NameKind {

	/**
	 * A name bound in the scope, and not captured by nested scopes. Inside a
	 * function, this is a local variable of the function; inside a class body,
	 * this is an attribute of the class.
	 */
	LOCAL,

	/**
	 * A name bound in a function, and captured by a nested scope (that refers
	 * to it as {@link #FREE}).
	 */
	CELL,

	/**
	 * A name that refers to a {@link #CELL} of an enclosing function, either
	 * implicitly (it is read but not bound in the scope) or explicitly (through
	 * a {@code nonlocal} declaration).
	 */
	FREE,

	/**
	 * A name declared {@code global} in the scope: it is looked up and bound in
	 * the namespace of the module (falling back to builtins when read).
	 */
	GLOBAL_EXPLICIT,

	/**
	 * A name that is read but not bound in the scope, and that is not bound in
	 * any enclosing function: it is looked up in the namespace of the module
	 * (falling back to builtins). All the names of the module scope are of this
	 * kind, unless declared {@code global}.
	 */
	GLOBAL_IMPLICIT;

	/**
	 * Yields whether this kind denotes a name of the namespace of the module.
	 *
	 * @return whether this is {@link #GLOBAL_EXPLICIT} or
	 *             {@link #GLOBAL_IMPLICIT}
	 */
	public boolean isGlobal() {
		return this == GLOBAL_EXPLICIT || this == GLOBAL_IMPLICIT;
	}

	/**
	 * Yields whether this kind denotes a name stored in a cell, that is shared
	 * between a function and the scopes nested into it.
	 *
	 * @return whether this is {@link #CELL} or {@link #FREE}
	 */
	public boolean isCell() {
		return this == CELL || this == FREE;
	}
}
