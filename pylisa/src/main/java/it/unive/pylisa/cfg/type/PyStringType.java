package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.StringType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Collection;
import java.util.Set;

public class PyStringType implements StringType {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyStringType INSTANCE = new PyStringType();

	private PyStringType() {
	}
	@Override
	public String toString() {
		return "String";
	}
	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyStringType|| other instanceof Untyped;
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
