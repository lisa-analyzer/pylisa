package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Set;

public class LiSAHttpServiceEndpoint implements Type {
	public static final LiSAHttpServiceEndpoint INSTANCE = new LiSAHttpServiceEndpoint();

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return false;
	}

	@Override
	public Type commonSupertype(
			Type other) {
		return null;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		return Set.of();
	}

	@Override
	public String toString() {
		return "$LiSAHttpServiceEndpoint";
	}
}
