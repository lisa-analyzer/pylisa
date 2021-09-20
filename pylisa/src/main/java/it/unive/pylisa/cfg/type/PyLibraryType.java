package it.unive.pylisa.cfg.type;


import it.unive.lisa.type.Type;

import java.util.Collection;
import java.util.Set;

public class PyLibraryType implements Type {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyLibraryType INSTANCE = new PyLibraryType();

	private PyLibraryType() {
	}
	@Override
	public String toString() {
		return "Library";
	}
	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyLibraryType;
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



}
