package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
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

	public static PyClassType lookup(String name) {
		return lookup(name, null);
	}

	public static PyClassType lookup(String name, CompilationUnit unit) {
		return types.computeIfAbsent(name, x -> new PyClassType(name, unit));
	}

	private final String name;

	private Integer hash = null;

	private final CompilationUnit unit;

	protected PyClassType(String name, CompilationUnit unit) {
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
	public final boolean canBeAssignedTo(Type other) {
		return other instanceof PyClassType && subclass((PyClassType) other);
	}

	private boolean subclass(PyClassType other) {
		return this == other || unit.isInstanceOf(other.unit);
	}

	@Override
	public Type commonSupertype(Type other) {
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

	private Type scanForSupertypeOf(UnitType other) {
		WorkingSet<PyClassType> ws = FIFOWorkingSet.mk();
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
			current.unit.getSuperUnits().forEach(u -> ws.push(lookup(u.getName(), null)));
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
	public boolean equals(Object obj) {
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
	public Collection<Type> allInstances() {
		Collection<Type> instances = new HashSet<>();
		for (CompilationUnit in : unit.getInstances())
			instances.add(lookup(in.getName(), null));
		return instances;
	}
}