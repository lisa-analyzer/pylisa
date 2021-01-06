package it.unive.pylisa.cfg.type;

import it.unive.lisa.cfg.type.BooleanType;
import it.unive.lisa.cfg.type.NumericType;
import it.unive.lisa.cfg.type.Type;

public class PyIntType implements NumericType{
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyIntType INSTANCE = new PyIntType();

	private PyIntType() {
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Type commonSupertype(Type other) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean is8Bits() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean is16Bits() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean is32Bits() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean is64Bits() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isUnsigned() {
		// TODO Auto-generated method stub
		return false;
	}

	
	
}
