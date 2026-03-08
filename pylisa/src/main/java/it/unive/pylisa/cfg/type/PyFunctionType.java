package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.*;
import it.unive.pylisa.program.FunctionUnit;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class PyFunctionType implements UnitType {
	private static final Map<String, PyFunctionType> types = new HashMap<>();

	public static PyFunctionType register(
			String name,
			FunctionUnit unit) {
		return types.computeIfAbsent(name, x -> new PyFunctionType(name, unit));
	}

	public static boolean isRegistered(
			String name) {
		return types.containsKey(name);
	}

	public static PyFunctionType lookup(
			String name) {
		PyFunctionType t = types.get(name);
		if (t == null)
			throw new IllegalStateException("Module type '" + name + "' not registered");
		return t;
	}

	private final String name;
	private final FunctionUnit unit;

	private PyFunctionType(
			String name,
			FunctionUnit unit) {
		this.name = name;
		this.unit = unit;
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
		return name;
	}

	@Override
	public FunctionUnit getUnit() {
		return unit;
	}
}
