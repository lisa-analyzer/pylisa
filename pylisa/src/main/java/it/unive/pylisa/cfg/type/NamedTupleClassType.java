package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * The type of the classes created by {@code collections.namedtuple}, whose name
 * and fields are known (see {@code NamedTupleBuiltin}): calling such a class
 * creates a tuple whose elements are also its attributes, named by the fields
 * (see {@code NamedTuples}). Two classes created at different locations are
 * different classes.
 */
public class NamedTupleClassType implements Type {

	private final String name;

	private final List<String> fields;

	private final CodeLocation location;

	/**
	 * Builds the type.
	 *
	 * @param name     the name of the class
	 * @param fields   the names of the fields, in order
	 * @param location the location where the class is created
	 */
	public NamedTupleClassType(
			String name,
			List<String> fields,
			CodeLocation location) {
		this.name = name;
		this.fields = List.copyOf(fields);
		this.location = location;
	}

	/**
	 * Yields the name of the class.
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	/**
	 * Yields the names of the fields, in order.
	 *
	 * @return the fields
	 */
	public List<String> getFields() {
		return fields;
	}

	/**
	 * Yields the location where the class is created.
	 *
	 * @return the location
	 */
	public CodeLocation getLocation() {
		return location;
	}

	@Override
	public boolean canBeAssignedTo(
			Type other) {
		return equals(other) || other.isUntyped();
	}

	@Override
	public Type commonSupertype(
			Type other) {
		return equals(other) ? this : it.unive.lisa.type.Untyped.INSTANCE;
	}

	@Override
	public Set<Type> allInstances(
			TypeSystem types) {
		return Collections.singleton(this);
	}

	@Override
	public String toString() {
		return "namedtuple::" + name + fields + "@" + location;
	}

	@Override
	public int hashCode() {
		return Objects.hash(name, fields, location);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null || getClass() != obj.getClass())
			return false;
		NamedTupleClassType other = (NamedTupleClassType) obj;
		return name.equals(other.name) && fields.equals(other.fields) && location.equals(other.location);
	}
}
