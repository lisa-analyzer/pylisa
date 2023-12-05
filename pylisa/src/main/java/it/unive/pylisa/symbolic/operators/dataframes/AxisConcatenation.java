package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;
import java.util.Collections;
import java.util.Set;

public class AxisConcatenation implements UnaryOperator, DataframeOperator {

	private final Axis axis;

	private final int index;

	public AxisConcatenation(
			int index,
			Axis axis) {
		this.index = index;
		this.axis = axis;
	}

	@Override
	public int getIndex() {
		return index;
	}

	public Axis getAxis() {
		return axis;
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> arg) {
		PyClassType list = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (arg.stream().noneMatch(t -> t.equals(list)))
			return Collections.emptySet();
		return Collections.singleton(df);
	}

	@Override
	public String toString() {
		return "concat(" + axis + ")";
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((axis == null) ? 0 : axis.hashCode());
		return result;
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
		AxisConcatenation other = (AxisConcatenation) obj;
		if (axis != other.axis)
			return false;
		return true;
	}
}
