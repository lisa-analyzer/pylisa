package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.caches.Caches;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull.Axis;

public class AxisConcatenation implements UnaryOperator {

	private final Axis axis;

	public AxisConcatenation(Axis axis) {
		this.axis = axis;
	}

	public Axis getAxis() {
		return axis;
	}

	@Override
	public ExternalSet<Type> typeInference(ExternalSet<Type> arg) {
		PyClassType list = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		PyClassType df = PyClassType.lookup(LibrarySpecificationProvider.PANDAS_DF);
		if (arg.noneMatch(t -> t.equals(list)))
			return Caches.types().mkEmptySet();
		return Caches.types().mkSingletonSet(df);
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
	public boolean equals(Object obj) {
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
