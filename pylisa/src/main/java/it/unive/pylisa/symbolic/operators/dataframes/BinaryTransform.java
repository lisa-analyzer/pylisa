package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.Enumerations.BinaryTransformKind;
import java.util.Collections;
import java.util.Optional;
import java.util.Set;

public class BinaryTransform implements BinaryOperator, DataframeOperator {

	private final BinaryTransformKind type;

	private final Axis axis;

	private final Optional<Object> arg;

	private final int index;

	public BinaryTransform(int index, BinaryTransformKind type, Axis axis) {
		this(index, type, axis, null);
	}

	public BinaryTransform(int index, BinaryTransformKind type, Axis axis, Object arg) {
		this.index = index;
		this.type = type;
		this.axis = axis;
		this.arg = Optional.ofNullable(arg);
	}

	@Override
	public int getIndex() {
		return index;
	}

	public BinaryTransformKind getKind() {
		return type;
	}

	public Axis getAxis() {
		return axis;
	}

	public Optional<Object> getArg() {
		return arg;
	}

	@Override
	public String toString() {
		if (arg.isEmpty())
			return "apply(" + type + " on " + axis + ")";
		else
			return "apply(" + type + " on " + axis + ", " + arg.get() + ")";
	}

	@Override
	public Set<Type> typeInference(TypeSystem types, Set<Type> left, Set<Type> right) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (left.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
		// TODO do we need conditions on right? We might have an expected type
		// in the constructor...
		return Collections.singleton(series);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
		result = prime * result + ((axis == null) ? 0 : axis.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		BinaryTransform other = (BinaryTransform) obj;
		if (arg == null) {
			if (other.arg != null)
				return false;
		} else if (!arg.equals(other.arg))
			return false;
		if (type != other.type)
			return false;
		if (axis != other.axis)
			return false;
		return true;
	}
}
