package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.DataframeSelection;

public class AssignValue extends DataframeOperation {

	private DataframeSelection selection;
	private int value;

	public AssignValue(CodeLocation where) {
		super(where);
	}

	@Override
	protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
		AssignValue o = (AssignValue) other;
		return this.equals(o);
	}

	@Override
	protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
		if (lessOrEqual(other)) {
			return this;
		}
		return DataframeOperation.TOP;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		AssignValue other = (AssignValue) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		return value == other.value;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		return result;
	}

	@Override
	public String toString() {
		return "assign(" + selection + ", " + value + ")";
	}

}
