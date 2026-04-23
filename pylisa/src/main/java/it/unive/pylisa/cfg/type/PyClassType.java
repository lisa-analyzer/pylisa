package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.lisa.util.collections.workset.WorkingSet;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

public class PyClassType implements InMemoryType, UnitType {

	protected static final Map<String, PyClassType> types = new HashMap<>();

	public static void clearAll() {
		types.clear();
	}

	public static Collection<PyClassType> all() {
		return types.values();
	}

	public static boolean isRegistered(
			String name) {
		return types.containsKey(name);
	}

	public static PyClassType lookup(
			String name) {
		PyClassType ct = types.get(name);
		if (ct == null)
			throw new IllegalStateException("The requested type '" + name + "' has not been registered before");

		return ct;
	}

	public static PyClassType register(
			String name,
			CompilationUnit unit) {
		return types.computeIfAbsent(name, x -> new PyClassType(name, unit));
	}

	/**
	 * Yields every registered class type whose backing {@link PyClassUnit} has
	 * the given {@code baseName} — i.e. the Python-visible qualified name
	 * without any allocation-site suffix. Two conditionally-declared classes
	 * with the same textual name share a base name but have distinct identity
	 * names, so this method is how language-level name resolution (imports,
	 * attribute access, inheritance) collects all def-sites at a given name.
	 * <p>
	 * Units that are not {@link it.unive.pylisa.program.PyClassUnit} (library
	 * specs loaded as plain {@link CompilationUnit}) match only when their
	 * registered name equals {@code baseName} exactly.
	 *
	 * @param baseName the Python-visible qualified name
	 *
	 * @return all types registered under that base name; never {@code null}
	 */
	public static Collection<PyClassType> lookupAllByBaseName(
			String baseName) {
		Collection<PyClassType> result = new java.util.ArrayList<>();
		for (PyClassType t : types.values()) {
			String candidate = (t.unit instanceof it.unive.pylisa.program.PyClassUnit pcu)
					? pcu.getBaseName()
					: t.name;
			if (baseName.equals(candidate))
				result.add(t);
		}
		return result;
	}

	/**
	 * Yields the single class type with the given base name, or throws when the
	 * base name is ambiguous. Intended for callers that have a prior guarantee
	 * of uniqueness (library specs, synthetic classes).
	 *
	 * @param baseName the Python-visible qualified name
	 *
	 * @return the unique class type with that base name
	 *
	 * @throws IllegalStateException when zero or multiple matches exist
	 */
	public static PyClassType lookupByBaseName(
			String baseName) {
		Collection<PyClassType> matches = lookupAllByBaseName(baseName);
		if (matches.isEmpty())
			throw new IllegalStateException(
					"No class type registered for base name '" + baseName + "'");
		if (matches.size() > 1)
			throw new IllegalStateException(
					"Base name '" + baseName + "' is ambiguous (" + matches.size()
							+ " def-sites); use lookupAllByBaseName instead");
		return matches.iterator().next();
	}

	private final String name;

	private Integer hash = null;

	private final CompilationUnit unit;

	protected PyClassType(
			String name,
			CompilationUnit unit) {
		Objects.requireNonNull(name, "The name of a class type cannot be null");
		Objects.requireNonNull(unit, "The unit of a class type cannot be null");
		this.name = name;
		this.unit = unit;
	}

	@Override
	public CompilationUnit getUnit() {
		return unit;
	}

	public ReferenceType getReference() {
		return new ReferenceType(this);
	}

	@Override
	public final boolean canBeAssignedTo(
			Type other) {
		return other instanceof PyClassType && subclass((PyClassType) other);
	}

	private boolean subclass(
			PyClassType other) {
		return this == other || unit.isInstanceOf(other.unit);
	}

	@Override
	public Type commonSupertype(
			Type other) {
		if (other.isNullType())
			return this;

		if (!other.isUnitType())
			return Untyped.INSTANCE;

		if (canBeAssignedTo(other))
			return other;

		if (other.canBeAssignedTo(this))
			return this;

		return scanForSupertypeOf((UnitType) other);
	}

	private Type scanForSupertypeOf(
			UnitType other) {
		WorkingSet<PyClassType> ws = new FIFOWorkingSet<>();
		Set<PyClassType> seen = new HashSet<>();
		ws.push(this);
		PyClassType current;
		while (!ws.isEmpty()) {
			current = ws.pop();
			if (!seen.add(current))
				continue;

			if (other.canBeAssignedTo(current))
				return current;

			// null since we do not want to create new types here
			current.unit.getImmediateAncestors().forEach(u -> ws.push(lookup(u.getName())));
		}

		return Untyped.INSTANCE;
	}

	@Override
	public String toString() {
		return name;
	}

	@Override
	public int hashCode() {
		if (hash != null)
			return hash;
		final int prime = 31;
		int result = 1;
		result = prime * result + ((name == null) ? 0 : name.hashCode());
		result = prime * result + ((unit == null) ? 0 : unit.hashCode());
		hash = result;
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		PyClassType other = (PyClassType) obj;
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		if (unit == null) {
			if (other.unit != null)
				return false;
		} else if (!unit.equals(other.unit))
			return false;
		return true;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		Set<Type> instances = new HashSet<>();
		for (Unit in : unit.getInstances())
			instances.add(lookup(in.getName()));
		return instances;
	}
}