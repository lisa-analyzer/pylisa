package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Objects;

public class Field {

	private final boolean instance;
	private final String name;
	private final Type type;
	private final Value initialValue;

	public Field(
			boolean instance,
			String name,
			Type type) {
		this(instance, name, type, null);
	}

	/**
	 * Builds a field that the initialization of its library module sets to
	 * the given value.
	 *
	 * @param instance     whether the field belongs to instances
	 * @param name         the name of the field
	 * @param type         the type of the field
	 * @param initialValue the value of the field after the initialization of
	 *                         the module, or {@code null} if the module does
	 *                         not set it
	 */
	public Field(
			boolean instance,
			String name,
			Type type,
			Value initialValue) {
		this.instance = instance;
		this.name = name;
		this.type = type;
		this.initialValue = initialValue;
	}

	/**
	 * Yields the value of the field after the initialization of its library
	 * module.
	 *
	 * @return the value, or {@code null} if the module does not set it
	 */
	public Value getInitialValue() {
		return initialValue;
	}

	public boolean isInstance() {
		return instance;
	}

	public String getName() {
		return name;
	}

	public Type getType() {
		return type;
	}

	@Override
	public int hashCode() {
		return Objects.hash(instance, name, type);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Field other = (Field) obj;
		return instance == other.instance && Objects.equals(name, other.name) && Objects.equals(type, other.type);
	}

	@Override
	public String toString() {
		return "Field [instance=" + instance + ", name=" + name + ", type=" + type + "]";
	}

	public Global toLiSAObject(
			CodeLocation location,
			Unit container) {
		it.unive.lisa.type.Type type = this.type.toLiSAType();
		return new Global(location, container, name, this.instance, type);
	}
}
