package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.Type;

import java.util.Collection;

public class PyArrayType implements Type {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyArrayType INSTANCE = new PyArrayType();

	private PyArrayType() {
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
