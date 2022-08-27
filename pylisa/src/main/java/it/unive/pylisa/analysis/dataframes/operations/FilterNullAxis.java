package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull.Axis;

public class FilterNullAxis extends DataframeOperation {

	private final Axis axis;

	public FilterNullAxis(CodeLocation where, Axis axis) {
		super(where);
		this.axis = axis;
	}

	public Axis getAxis() {
		return axis;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) {
		FilterNullAxis o = (FilterNullAxis) other;
		if (axis == Axis.TOP)
			return o.axis == Axis.TOP;

		if (o.axis == Axis.TOP)
			return true;

		return o.axis == axis;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) {
		FilterNullAxis o = (FilterNullAxis) other;
		Axis ax;
		if (axis == Axis.TOP || o.axis == Axis.TOP || axis != o.axis)
			ax = Axis.TOP;
		else
			ax = axis;

		return new FilterNullAxis(loc(other), ax);
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
		FilterNullAxis other = (FilterNullAxis) obj;
		if (axis != other.axis)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "filter_null(" + axis + ")";
	}

	@Override
	protected int compareToSameClassAndLocation(DataframeOperation o) {
		FilterNullAxis other = (FilterNullAxis) o;
		return axis.compareTo(other.axis);
	}
}
