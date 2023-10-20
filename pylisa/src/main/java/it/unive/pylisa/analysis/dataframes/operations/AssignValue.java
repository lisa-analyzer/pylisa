package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.operations.selection.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.RowSelection;

public class AssignValue<R extends RowSelection<R>, C extends ColumnSelection<C>> extends DataframeOperation {

	private final DataframeSelection<R, C> selection;
	private final ConstantPropagation value;

	public AssignValue(
			CodeLocation where,
			DataframeSelection<R, C> selection,
			ConstantPropagation value) {
		super(where);
		this.selection = selection;
		this.value = value;
	}

	public DataframeSelection<R, C> getSelection() {
		return selection;
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException {
		if (!(other instanceof AssignValue))
			return false;
		@SuppressWarnings("unchecked")
		AssignValue<R, C> o = (AssignValue<R, C>) other;
		return selection.lessOrEqual(o.selection) && value.lessOrEqual(o.value);
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException {
		if (lessOrEqual(other)) {
			return this;
		}
		return DataframeOperation.TOP;
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
		AssignValue<?, ?> other = (AssignValue<?, ?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;

		if (value == null) {
			if (other.value != null)
				return false;
		} else if (!value.equals(other.value))
			return false;

		return true;
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

	@Override
	protected int compareToSameClassAndLocation(
			DataframeOperation o) {
		AssignValue<?, ?> other = (AssignValue<?, ?>) o;
		int cmp = selection.compareTo(other.selection);
		if (cmp != 0)
			return cmp;
		return value.compareTo(other.value);
	}

}
