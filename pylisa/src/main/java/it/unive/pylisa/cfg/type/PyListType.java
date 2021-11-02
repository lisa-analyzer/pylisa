package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Collection;
import java.util.Set;

public class PyListType implements Type {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyListType INSTANCE = new PyListType();

	private PyListType() {
	}

	@Override
	public String toString() {
		return "List";
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyListType|| other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(Type other) {
		if(other==this)
			return this;
		else return  Untyped.INSTANCE;
	}

	@Override
	public Collection<Type> allInstances() {
		return Set.of(INSTANCE);
	}



}
