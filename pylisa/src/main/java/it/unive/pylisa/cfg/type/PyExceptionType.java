package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * The type of a Python exception, identified by the qualified name of its class
 * and placed in the hierarchy of exception classes through its superclass.
 * <p>
 * Exception types are the types of the errors an analysis can raise: an error
 * of type {@code T} is caught by a handler for any ancestor of {@code T}. They
 * are independent from the classes of the analysed program, so that library
 * models can raise the exceptions of their library even when the program never
 * imports the module that defines them.
 * </p>
 * <p>
 * Instances are created once, as constants, and never change; the list of
 * direct subclasses of a type only grows while those constants are created.
 * </p>
 */
public final class PyExceptionType implements Type {

	/**
	 * {@code BaseException}, the root of all Python exceptions.
	 */
	public static final PyExceptionType BASE_EXCEPTION = new PyExceptionType("builtins.BaseException", null);

	/**
	 * {@code Exception}, the root of all non-exit exceptions.
	 */
	public static final PyExceptionType EXCEPTION = subclass("builtins.Exception", BASE_EXCEPTION);

	/**
	 * {@code AssertionError}, raised by a failing {@code assert}.
	 */
	public static final PyExceptionType ASSERTION_ERROR = subclass("builtins.AssertionError", EXCEPTION);

	/**
	 * {@code TypeError}, raised when an operation receives a value of the wrong
	 * type.
	 */
	public static final PyExceptionType TYPE_ERROR = subclass("builtins.TypeError", EXCEPTION);

	/**
	 * {@code ValueError}, raised when an operation receives a value of the
	 * right type but of an unacceptable value.
	 */
	public static final PyExceptionType VALUE_ERROR = subclass("builtins.ValueError", EXCEPTION);

	/**
	 * {@code KeyboardInterrupt}, raised when the process receives SIGINT.
	 */
	public static final PyExceptionType KEYBOARD_INTERRUPT = subclass("builtins.KeyboardInterrupt", BASE_EXCEPTION);

	/**
	 * {@code RuntimeError}, raised for errors that fit no other category.
	 */
	public static final PyExceptionType RUNTIME_ERROR = subclass("builtins.RuntimeError", EXCEPTION);

	/**
	 * {@code NotImplementedError}, raised by abstract methods.
	 */
	public static final PyExceptionType NOT_IMPLEMENTED_ERROR = subclass("builtins.NotImplementedError",
			RUNTIME_ERROR);

	/**
	 * {@code AttributeError}, raised when an attribute an object does not have
	 * is used.
	 */
	public static final PyExceptionType ATTRIBUTE_ERROR = subclass("builtins.AttributeError", EXCEPTION);

	/**
	 * {@code ArithmeticError}.
	 */
	public static final PyExceptionType ARITHMETIC_ERROR = subclass("builtins.ArithmeticError", EXCEPTION);

	/**
	 * {@code OverflowError}, raised for instance when an infinite float is
	 * converted to an integer.
	 */
	public static final PyExceptionType OVERFLOW_ERROR = subclass("builtins.OverflowError", ARITHMETIC_ERROR);

	/**
	 * {@code ZeroDivisionError}, raised by a division or a remainder by zero.
	 */
	public static final PyExceptionType ZERO_DIVISION_ERROR = subclass("builtins.ZeroDivisionError",
			ARITHMETIC_ERROR);

	/**
	 * {@code SystemExit}, raised by {@code sys.exit()} and by a command-line
	 * parser that rejects its arguments.
	 */
	public static final PyExceptionType SYSTEM_EXIT = subclass("builtins.SystemExit", BASE_EXCEPTION);

	/**
	 * The exception classes of the {@code builtins} module that have a type
	 * here, by their unqualified name: every class of the hierarchy under
	 * {@link #BASE_EXCEPTION} named in {@code builtins}. A holder class, so
	 * that the map is built after the constants it lists.
	 */
	private static final class Builtins {

		private static final String MODULE = "builtins.";

		private static final Map<String, PyExceptionType> BY_NAME = byName();

		private static Map<String, PyExceptionType> byName() {
			Map<String, PyExceptionType> map = new HashMap<>();
			Deque<PyExceptionType> pending = new ArrayDeque<>(List.of(BASE_EXCEPTION));
			while (!pending.isEmpty()) {
				PyExceptionType type = pending.pop();
				if (type.name.startsWith(MODULE))
					map.put(type.name.substring(MODULE.length()), type);
				pending.addAll(type.subclasses);
			}
			return Collections.unmodifiableMap(map);
		}
	}

	/**
	 * Yields the type of an exception class of the {@code builtins} module.
	 *
	 * @param name the unqualified name of the class, such as
	 *                 {@code TypeError}
	 *
	 * @return the type, or {@code null} if the class has no type here
	 */
	public static PyExceptionType builtin(
			String name) {
		return Builtins.BY_NAME.get(name);
	}

	private final String name;

	private final PyExceptionType superclass;

	private final List<PyExceptionType> subclasses = new CopyOnWriteArrayList<>();

	private PyExceptionType(
			String name,
			PyExceptionType superclass) {
		this.name = Objects.requireNonNull(name);
		this.superclass = superclass;
		if (superclass != null)
			superclass.subclasses.add(this);
	}

	/**
	 * Builds the type of an exception class. Each exception class must be
	 * built once, and stored in a constant.
	 *
	 * @param name       the qualified name of the class, such as
	 *                       {@code mylib.errors.InvalidNameError}
	 * @param superclass the type of the direct superclass
	 *
	 * @return the type
	 */
	public static PyExceptionType subclass(
			String name,
			PyExceptionType superclass) {
		return new PyExceptionType(name, Objects.requireNonNull(superclass));
	}

	/**
	 * Yields the qualified name of the exception class.
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * Yields the type of the direct superclass.
	 *
	 * @return the superclass type, or {@code null} for
	 *             {@link #BASE_EXCEPTION}
	 */
	public PyExceptionType getSuperclass() {
		return superclass;
	}

	/**
	 * Yields whether this type is {@code other} or one of its subclasses.
	 *
	 * @param other the candidate ancestor
	 *
	 * @return {@code true} if an exception of this type is an instance of
	 *             {@code other}
	 */
	public boolean isSubclassOf(
			PyExceptionType other) {
		for (PyExceptionType current = this; current != null; current = current.superclass)
			if (current == other)
				return true;
		return false;
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return other.isUntyped() || (other instanceof PyExceptionType type && isSubclassOf(type));
	}

	@Override
	public Type commonSupertype(
			Type other) {
		if (other instanceof PyExceptionType type)
			for (PyExceptionType candidate = this; candidate != null; candidate = candidate.superclass)
				if (type.isSubclassOf(candidate))
					return candidate;
		return Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		Set<Type> instances = new HashSet<>();
		List<PyExceptionType> pending = new ArrayList<>(List.of(this));
		while (!pending.isEmpty()) {
			PyExceptionType current = pending.remove(pending.size() - 1);
			if (instances.add(current))
				pending.addAll(current.subclasses);
		}
		return Collections.unmodifiableSet(instances);
	}

	@Override
	public String toString() {
		return name;
	}
}
