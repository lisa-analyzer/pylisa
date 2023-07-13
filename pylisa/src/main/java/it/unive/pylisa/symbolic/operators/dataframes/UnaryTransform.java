package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.Enumerations.UnaryTransformKind;
import java.util.Collections;
import java.util.Optional;
import java.util.Set;

public class UnaryTransform implements UnaryOperator, DataframeOperator {

	private final UnaryTransformKind type;

	private final Axis axis;

	private final Optional<Object> arg;

	private final int index;

	@Override
	public int getIndex() {
		return index;
	}

	public UnaryTransform(int index, UnaryTransformKind type, Axis axis) {
		this(index, type, axis, null);
	}

	public UnaryTransform(int index, UnaryTransformKind type, Axis axis, Object arg) {
		this.index = index;
		this.type = type;
		this.axis = axis;
		this.arg = Optional.ofNullable(arg);
	}

	public UnaryTransformKind getKind() {
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
	public Set<Type> typeInference(TypeSystem types, Set<Type> argument) {
		PyClassType series = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_SERIES);
		if (argument.stream().noneMatch(t -> t.equals(series)))
			return Collections.emptySet();
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
		UnaryTransform other = (UnaryTransform) obj;
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
