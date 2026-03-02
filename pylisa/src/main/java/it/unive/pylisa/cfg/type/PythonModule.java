package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.PyCFG;
import java.util.Set;

public class PythonModule implements Type {
	private final PyCFG initFunction;
	private final String name;

	public PythonModule(
			PyCFG initFunction,
			String name) {
		this.initFunction = initFunction;
		this.name = name;
	}

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
}
