package it.unive.pylisa.symbolic.operators.dataframes;

import java.util.Collections;
import java.util.Optional;
import java.util.Set;

import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import it.unive.pylisa.symbolic.operators.Enumerations.BinaryKind;

public class BinaryTransform implements BinaryOperator {

	private final BinaryKind type;

	private final Axis axis;

	private final Optional<Object> arg;

	private final boolean changeShape;

	public BinaryTransform(BinaryKind type, Axis axis, boolean changeShape) {
		this(type, axis, changeShape, null);
	}

	public BinaryTransform(BinaryKind type, Axis axis, boolean changeShape, Object arg) {
		this.type = type;
		this.axis = axis;
		this.changeShape = changeShape;
		this.arg = Optional.ofNullable(arg);
	}

	public BinaryKind getKind() {
		return type;
	}
	
	public Axis getAxis() {
		return axis;
	}

	public Optional<Object> getArg() {
		return arg;
	}
	
	public boolean isChangeShape() {
		return changeShape;
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
		// TODO do we need conditions on right? We might have an expected type in the constructor...
		return Collections.singleton(series);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((arg == null) ? 0 : arg.hashCode());
		result = prime * result + (changeShape ? 1231 : 1237);
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
		if (changeShape != other.changeShape)
			return false;
		if (type != other.type)
			return false;
		if (axis != other.axis)
			return false;
		return true;
	}
}
