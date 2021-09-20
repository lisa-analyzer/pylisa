package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.Type;

import java.util.Collection;
import java.util.Set;

public class PyIntType implements NumericType {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyIntType INSTANCE = new PyIntType();

	private PyIntType() {
	}

	@Override
	public String toString() {
		return "int";
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof NumericType;
	}

	@Override
	public Type commonSupertype(Type other) {
		if(other==this)
			return this;
		else return  PyTopType.INSTANCE;
	}

	@Override
	public Collection<Type> allInstances() {
		return Set.of(INSTANCE);
	}
	@Override
	public boolean is8Bits() {
		return false;
	}

	@Override
	public boolean is16Bits() {
		return false;
	}

	@Override
	public boolean is32Bits() {
		return true;
	}

	@Override
	public boolean is64Bits() {
		return false;
	}

	@Override
	public boolean isUnsigned() {
		return false;
	}

	@Override
	public boolean isIntegral() {
		return true;
	}

}
