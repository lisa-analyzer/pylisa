package it.unive.pylisa.cfg.type;

import it.unive.lisa.cfg.type.BooleanType;
import it.unive.lisa.cfg.type.Type;

public class PyBoolType implements BooleanType{
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyBoolType INSTANCE = new PyBoolType();

	private PyBoolType() {
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other.isBooleanType() || other.isUntyped();
	}

	@Override
	public Type commonSupertype(Type other) {
		return other.isBooleanType() ? this : Untyped.INSTANCE;
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
	
}
