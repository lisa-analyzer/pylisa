package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.symbolic.operators.dataframes.FillNull.Axis;

public class FillNullAxis extends DataframeOperation {

	private final Axis axis;
	private final ConstantPropagation value;

	public FillNullAxis(CodeLocation where, Axis axis, ConstantPropagation value) {
		super(where);
		this.axis = axis;
		this.value = value;
	}

	public Axis getAxis() {
		return axis;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		FillNullAxis o = (FillNullAxis) other;
		if (!value.lessOrEqual(o.value))
			return false;

		if (axis == Axis.TOP)
			return o.axis == Axis.TOP;

		if (o.axis == Axis.TOP)
			return true;

		return o.axis == axis;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		FillNullAxis o = (FillNullAxis) other;
		Axis ax;
		if (axis == Axis.TOP || o.axis == Axis.TOP || axis != o.axis)
			ax = Axis.TOP;
		else
			ax = axis;

		return new FillNullAxis(loc(other), ax, value.lub(o.value));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((axis == null) ? 0 : axis.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		FillNullAxis other = (FillNullAxis) obj;
		if (axis != other.axis)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "fill_null(" + axis + ", " + value + ")";
	}

	@Override
	protected int compareToSameClassAndLocation(DataframeOperation o) {
		FillNullAxis other = (FillNullAxis) o;
		return axis.compareTo(other.axis);
	}
}
