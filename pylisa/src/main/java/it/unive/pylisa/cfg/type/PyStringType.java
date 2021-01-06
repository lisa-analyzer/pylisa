package it.unive.pylisa.cfg.type;

import it.unive.lisa.cfg.type.BooleanType;
import it.unive.lisa.cfg.type.NumericType;
import it.unive.lisa.cfg.type.StringType;
import it.unive.lisa.cfg.type.Type;

public class PyStringType implements StringType{
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyStringType INSTANCE = new PyStringType();

	private PyStringType() {
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

	

	
	
}
