package it.unive.pylisa.cfg.type;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;

public class PyEllipsisType implements Type {

	public static final PyEllipsisType INSTANCE = new PyEllipsisType();

	protected PyEllipsisType() {
	}

	@Override
	public String toString() {
		return "Ellipsis";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PyEllipsisType;
	}

	@Override
	public int hashCode() {
		return PyEllipsisType.class.hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return equals(other);
	}

	@Override
	public Type commonSupertype(Type other) {
		return other == this ? this : Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(TypeSystem types) {
		return Collections.singleton(this);
	}
}
