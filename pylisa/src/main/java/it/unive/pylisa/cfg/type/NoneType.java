package it.unive.pylisa.cfg.type;

import java.util.Collection;
import java.util.Collections;

import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.Type;

public class NoneType implements InMemoryType {

	public static final NoneType INSTANCE = new NoneType();

	private NoneType() {
	}

	@Override
	public String toString() {
		return "none";
	}

	@Override
	public final boolean equals(Object other) {
		return other instanceof NoneType;
	}

	@Override
	public final int hashCode() {
		return NoneType.class.hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return true;
	}

	@Override
	public Type commonSupertype(Type other) {
		return other;
	}

	@Override
	public Collection<Type> allInstances() {
		return Collections.singleton(this);
	}
}
