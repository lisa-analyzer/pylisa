package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class PyModuleType implements UnitType {

	private static final Map<String, PyModuleType> types = new HashMap<>();

	public static PyModuleType register(
			String name,
			CompilationUnit unit) {
		PyModuleType current = types.get(name);
		if (current == null) {
			current = new PyModuleType(name, unit, false);
			types.put(name, current);
		} else {
			current.unit = unit;
			current.unknown = false;
		}
		return current;
	}

	public static PyModuleType registerUnknown(
			String name,
			CompilationUnit unit) {
		PyModuleType current = types.get(name);
		if (current == null) {
			current = new PyModuleType(name, unit, true);
			types.put(name, current);
		} else if (current.unknown) {
			current.unit = unit;
		}
		return current;
	}

	public static boolean isRegistered(
			String name) {
		return types.containsKey(name);
	}

	public static PyModuleType lookup(
			String name) {
		PyModuleType t = types.get(name);
		if (t == null)
			throw new IllegalStateException("Module type '" + name + "' not registered");
		return t;
	}

	private final String name;
	private CompilationUnit unit;
	private boolean unknown;

	private PyModuleType(
			String name,
			CompilationUnit unit,
			boolean unknown) {
		this.name = name;
		this.unit = unit;
		this.unknown = unknown;
	}

	public boolean isUnknown() {
		return unknown;
	}

	public CompilationUnit getUnit() {
		return unit;
	}

	public ReferenceType getReference() {
		return new ReferenceType(this);
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		// modules are only assignable to themselves
		return this.equals(other);
	}

	@Override
	public Type commonSupertype(
			Type other) {
		if (other.isNullType())
			return this;
		if (this.equals(other))
			return this;
		return Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		// Modules do not have subclasses
		return Set.of(this);
	}

	@Override
	public String toString() {
		return unknown ? "(U)" + name : name;
	}

}