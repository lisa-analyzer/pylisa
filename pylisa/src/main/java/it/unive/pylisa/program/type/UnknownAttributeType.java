package it.unive.pylisa.program.type;

import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class UnknownAttributeType implements InMemoryType {

	private static final Map<String, UnknownAttributeType> TYPES = new ConcurrentHashMap<>();

	public static UnknownAttributeType lookup(
			String qualifiedName) {
		return TYPES.computeIfAbsent(qualifiedName, UnknownAttributeType::new);
	}

	private final String qualifiedName;

	private UnknownAttributeType(
			String qualifiedName) {
		this.qualifiedName = qualifiedName;
	}

	public String getQualifiedName() {
		return qualifiedName;
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return equals(other);
	}

	@Override
	public Type commonSupertype(
			Type other) {
		if (other != null && equals(other))
			return this;
		return Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		return Collections.singleton(this);
	}

	@Override
	public String toString() {
		return "(U)" + qualifiedName;
	}

	@Override
	public int hashCode() {
		return qualifiedName.hashCode();
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!(obj instanceof UnknownAttributeType other))
			return false;
		return qualifiedName.equals(other.qualifiedName);
	}
}
