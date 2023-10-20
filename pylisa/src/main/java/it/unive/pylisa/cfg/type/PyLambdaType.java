package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import java.util.Set;

public class PyLambdaType implements Type {

	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyLambdaType INSTANCE = new PyLambdaType();

	private PyLambdaType() {
	}

	@Override
	public String toString() {
		return "lambda";
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return other instanceof PyLambdaType || other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(
			Type other) {
		if (other == this)
			return this;
		else
			return Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		return Set.of(INSTANCE);
	}
}
