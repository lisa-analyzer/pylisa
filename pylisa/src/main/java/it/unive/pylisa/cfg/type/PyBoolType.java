package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.BooleanType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Collection;
import java.util.Set;

public class PyBoolType implements BooleanType {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyBoolType INSTANCE = new PyBoolType();

	private PyBoolType() {
	}

	@Override
	public String toString() {
		return "bool";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof BooleanType;
	}

	@Override
	public int hashCode() {
		return BooleanType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof BooleanType|| other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(Type other) {
		if(other==this)
			return this;
		else return PyTopType.INSTANCE;
	}

	@Override
	public Collection<Type> allInstances() {
		return Set.of(INSTANCE);
	}

}
