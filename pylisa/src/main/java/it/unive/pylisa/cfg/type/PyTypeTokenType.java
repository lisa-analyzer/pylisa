package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeTokenType;
import it.unive.lisa.type.Untyped;
import java.util.Set;

public class PyTypeTokenType extends TypeTokenType {
	/**
	 * Builds the type token representing the given types.
	 *
	 * @param types the types
	 */
	public PyTypeTokenType(Set<Type> types) {
		super(types);
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return super.canBeAssignedTo(other) || other instanceof Untyped;
	}

}
