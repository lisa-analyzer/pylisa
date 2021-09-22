package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.BooleanType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Collection;
import java.util.Set;

public class PyTopType implements Type {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyTopType INSTANCE = new PyTopType();

	private PyTopType() {
	}

	@Override
	public String toString() {
		return "T";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PyTopType;
	}

	@Override
	public int hashCode() {
		return PyTopType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyTopType|| other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(Type other) {
		return this;
	}

	@Override
	public Collection<Type> allInstances() {
		return Set.of(INSTANCE);
	}

}
