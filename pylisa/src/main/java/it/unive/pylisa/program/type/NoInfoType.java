package it.unive.pylisa.program.type;

import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Set;

public class NoInfoType
		implements
		InMemoryType {

	/**
	 * Unique instance of {@link it.unive.lisa.type.NullType}.
	 */
	public static final NoInfoType INSTANCE = new NoInfoType();

	/**
	 * Builds the type. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected NoInfoType() {
	}

	@Override
	public String toString() {
		return "NO_INFO";
	}

	@Override
	public boolean equals(
			Object other) {
		return other instanceof it.unive.lisa.type.NullType;
	}

	@Override
	public int hashCode() {
		return it.unive.lisa.type.NullType.class.hashCode();
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return false;
	}

	@Override
	public Type commonSupertype(
			Type other) {
		return other != null && other.isInMemoryType() ? other : Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		return Collections.singleton(this);
	}

}
