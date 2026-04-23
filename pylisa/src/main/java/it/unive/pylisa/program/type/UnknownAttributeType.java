package it.unive.pylisa.program.type;

import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class UnknownAttributeType implements InMemoryType {

	private static final Logger LOG = LogManager.getLogger(UnknownAttributeType.class);

	private static final Map<String, UnknownAttributeType> TYPES = new ConcurrentHashMap<>();

	private static final AtomicInteger CREATED = new AtomicInteger(0);

	public static int getRegisteredCount() {
		return TYPES.size();
	}

	public static UnknownAttributeType lookup(
			String qualifiedName) {
		UnknownAttributeType existing = TYPES.get(qualifiedName);
		if (existing != null)
			return existing;
		UnknownAttributeType created = TYPES.computeIfAbsent(qualifiedName, UnknownAttributeType::new);
		int count = CREATED.incrementAndGet();
		if (count % 100 == 0 || count < 20)
			LOG.info("[UAT-TRACK] UnknownAttributeType created #{} total={} name={}", count, TYPES.size(),
					qualifiedName);
		return created;
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
