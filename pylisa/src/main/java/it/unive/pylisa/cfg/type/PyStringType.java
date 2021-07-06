package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.StringType;
import it.unive.lisa.type.Type;

import java.util.Collection;

public class PyStringType implements StringType {
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

	@Override
	public Collection<Type> allInstances() {
		return null;
	}


}
