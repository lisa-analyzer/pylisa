package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;

public class Concat extends DataframeOperation {

	private final Axis axis;

	public Concat(CodeLocation where, Axis axis) {
		super(where);
		this.axis = axis;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		Concat o = (Concat) other;
		if (o.axis != this.axis) {
			return false;
		}

		return true;
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		Concat o = (Concat) other;
		if (o.axis != this.axis) {
			return new Concat(loc(other), Axis.TOP);
		}

		return new Concat(loc(other), this.axis);
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
		Concat other = (Concat) obj;
		if (axis != other.axis)
			return false;
		return true;
	}

	@Override
	public String toString() {
		switch (this.axis) {
		case COLS:
			return "concat_cols";
		case ROWS:
			return "concat_rows";
		case BOTH:
		case TOP:
		default:
			return "concat_TOP";

		}
	}

	@Override
	protected int compareToSameClassAndLocation(DataframeOperation o) {
		Concat other = (Concat) o;
		return axis.compareTo(other.axis);
	}

}