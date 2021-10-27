package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Collection;
import java.util.Set;

public class PyDataframeType implements Type {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyDataframeType INSTANCE = new PyDataframeType();

	private PyDataframeType() {
	}

	@Override
	public String toString() {
		return "T";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PyDataframeType;
	}

	@Override
	public int hashCode() {
		return PyDataframeType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyDataframeType || other instanceof Untyped;
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
