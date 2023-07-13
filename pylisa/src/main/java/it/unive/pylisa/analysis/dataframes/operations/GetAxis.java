package it.unive.pylisa.analysis.dataframes.operations;

import java.util.Objects;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.symbolic.operators.Enumerations.Axis;

public class GetAxis extends DataframeOperation {

	private final Axis axis;

	public GetAxis(CodeLocation where, int index, Axis axis) {
		super(where, index);
		this.axis = axis;
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		return axis.lessOrEqual(((GetAxis) other).axis);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		return new GetAxis(where, index, axis.lub(((GetAxis) other).axis));
	}

	@Override
	protected DataframeOperation wideningSameOperation(DataframeOperation other) throws SemanticException {
		return new GetAxis(where, index, axis.widening(((GetAxis) other).axis));
	}

	@Override
	protected int compareToSameOperation(DataframeOperation o) {
		return axis.compareTo(((GetAxis) o).axis);
	}

	@Override
	public String toString() {
		return "get_axis:" + axis;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + Objects.hash(axis);
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
		GetAxis other = (GetAxis) obj;
		return axis == other.axis;
	}
}
