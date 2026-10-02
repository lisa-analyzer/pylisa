package it.unive.pylisa.checks;

/**
 * The known ways in which the Python analysis may miss executions of the
 * analysed program. While any of them applies, results are best-effort: a
 * fact the analysis reports holds for the executions it models, but some real
 * executions may not be modelled.
 */
public enum KnownGap {

	/**
	 * In {@code a.b(x)}, {@code a} is always passed as the first argument, also
	 * when {@code b} is a function stored in an attribute of {@code a} rather
	 * than a method of its class.
	 */
	RECEIVER_CHOSEN_BY_SYNTAX("a.b(x) always passes a as first argument, also when b is a function stored on a"),

	/**
	 * Reading a method through an object yields the plain function, without
	 * the object: a method stored in a variable and called later is called
	 * without its receiver.
	 */
	NO_BOUND_METHODS("reading a method through an object loses the object it is bound to"),

	/**
	 * Functions are identified by their qualified name: when a function is
	 * defined twice, only the first definition is analysed.
	 */
	FUNCTIONS_IDENTIFIED_BY_NAME("a function defined twice under the same name is analysed with its first body only"),

	/**
	 * The arguments of a call may be evaluated more than once, so their side
	 * effects may be analysed more than once.
	 */
	ARGUMENTS_EVALUATED_REPEATEDLY("call arguments may be evaluated more than once"),

	/**
	 * Calls to code the analysis does not know return an unknown value and are
	 * assumed not to modify anything.
	 */
	UNKNOWN_CALLS_HAVE_NO_EFFECT("calls to unknown code are assumed to modify nothing"),

	/**
	 * Attributes are looked up breadth-first among the ancestors of a class,
	 * not in method resolution order, and {@code super()} goes to the first
	 * base of the class instead of the next class in the method resolution
	 * order of the receiver.
	 */
	NO_METHOD_RESOLUTION_ORDER("attribute lookup and super() ignore Python's method resolution order"),

	/**
	 * Decorators the analysis does not know are assumed to return the decorated
	 * function unchanged.
	 */
	UNKNOWN_DECORATORS_ARE_IDENTITY("unknown decorators are assumed to return the decorated function unchanged"),

	/**
	 * Writes to attributes whose name is only known at run time
	 * ({@code setattr}, {@code __dict__}, {@code vars()}) are not modelled.
	 */
	DYNAMIC_ATTRIBUTE_WRITES_IGNORED("writes to attributes named at run time are not modelled"),

	/**
	 * Default values of parameters are evaluated at every call instead of once
	 * when the function is defined, so mutable defaults are not shared between
	 * calls.
	 */
	DEFAULTS_EVALUATED_PER_CALL("default parameter values are evaluated at each call instead of once"),

	/**
	 * Functions capture the values of the variables of enclosing functions,
	 * not the variables themselves, so later changes to those variables are
	 * not seen.
	 */
	CLOSURES_CAPTURE_VALUES("closures capture values instead of variables"),

	/**
	 * Callbacks registered with a library, to be invoked later by the library
	 * (for example by an event loop), are recorded but never executed.
	 */
	CALLBACKS_NOT_RUN("callbacks registered with libraries are never executed"),

	/**
	 * {@code import a.b} binds a variable named {@code a.b} instead of binding
	 * {@code a} and making {@code b} one of its attributes, so {@code a.b.X}
	 * is unknown; {@code import a.b as c} ignores the alias, and
	 * {@code import x, y} imports only {@code x}. {@code from a.b import X}
	 * works.
	 */
	DOTTED_IMPORTS("import a.b does not make a.b reachable as an attribute of a; aliases and multiple names are ignored"),

	/**
	 * Exception handlers are not modelled: the body of a {@code try} block is
	 * analysed as if no exception could be caught.
	 */
	EXCEPTION_HANDLERS_IGNORED("try/except handlers are not modelled"),

	/**
	 * Coroutines are analysed as ordinary functions that run to completion
	 * when called.
	 */
	COROUTINES_RUN_SYNCHRONOUSLY("async functions are analysed as if they ran synchronously when called"),

	/**
	 * Some expressions are parsed incorrectly: the {@code %} operator may drop
	 * the rest of the line, adjacent string literals keep only the first one,
	 * and in a chained comparison such as {@code a < b < c} only the first
	 * comparison is evaluated (the others are an unknown truth value, and
	 * their further operands are not evaluated).
	 */
	PARSING_DEFECTS("the % operator, adjacent string literals and chained comparisons are parsed incompletely"),

	/**
	 * A library parameter whose default value is not a literal (such as
	 * {@code qos_profile=qos_profile_services_default}) is declared with the
	 * default {@code None}, so a call that passes {@code None} explicitly is
	 * analysed as if it passed nothing.
	 */
	EXPLICIT_NONE_AS_DEFAULT("an explicit None for a library parameter with a non-literal default is taken as the default"),

	/**
	 * A call that cannot return normally, because every execution of it
	 * raises, is continued with an unknown result, so the code after it is
	 * analysed as reachable.
	 */
	RAISING_CALLS_CONTINUE("the code after a call that always raises is analysed as reachable, with an unknown result"),

	/**
	 * Numbers are plain values, not objects: arithmetic and comparisons on
	 * them follow Python's rules for {@code int}, {@code bool} and
	 * {@code float} but never dispatch to {@code __add__}, {@code __lt__} and
	 * the other special methods, so subclasses of the numeric types are
	 * treated as their base type. Integers that do not fit 64 bits are
	 * unknown.
	 */
	NUMBERS_ARE_PLAIN_VALUES("numbers are plain values: no special-method dispatch, integers beyond 64 bits are unknown");

	private final String description;

	KnownGap(
			String description) {
		this.description = description;
	}

	/**
	 * Yields a one-line description of the gap.
	 *
	 * @return the description
	 */
	public String getDescription() {
		return description;
	}
}
