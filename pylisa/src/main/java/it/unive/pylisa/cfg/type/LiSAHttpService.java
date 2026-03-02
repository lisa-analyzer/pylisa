package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Set;

public class LiSAHttpService implements Type {
	public static final LiSAHttpService INSTANCE = new LiSAHttpService();

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
		return "$LiSAHttpService";
	}
}
