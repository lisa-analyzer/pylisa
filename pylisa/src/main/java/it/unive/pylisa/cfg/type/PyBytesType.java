package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Set;

/**
 * The type of Python's {@code bytes}: immutable sequences of integers between 0
 * and 255, modeled as values (like {@code str}, and unlike lists). It is not
 * compatible with {@code str}: {@code b"a" + "a"} raises {@code TypeError}.
 */
public class PyBytesType implements Type {

	/**
	 * The singleton instance of this type.
	 */
	public static final PyBytesType INSTANCE = new PyBytesType();

	private PyBytesType() {
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return other instanceof PyBytesType || other.isUntyped();
	}

	@Override
	public Type commonSupertype(
			Type other) {
		return other instanceof PyBytesType ? this : Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		return Collections.singleton(this);
	}

	@Override
	public String toString() {
		return "bytes";
	}

	@Override
	public boolean equals(
			Object other) {
		return other instanceof PyBytesType;
	}

	@Override
	public int hashCode() {
		return PyBytesType.class.getName().hashCode();
	}
}
