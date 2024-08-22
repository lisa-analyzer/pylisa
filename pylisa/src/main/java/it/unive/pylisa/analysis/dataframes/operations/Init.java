package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.NumberSlice;
import java.util.Objects;

public class Init extends DataframeOperation {

	private final Names knownColumns;

	private final NumberSlice existingRows;

	public Init(
			CodeLocation where,
			int index,
			Names knownColumns,
			NumberSlice existingRows) {
		super(where, index);
		this.knownColumns = knownColumns;
		this.existingRows = existingRows;
	}

	public Names getKnownColumns() {
		return knownColumns;
	}

	public NumberSlice getExistingRows() {
		return existingRows;
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Init o = (Init) other;
		return knownColumns.lessOrEqual(o.knownColumns) && existingRows.lessOrEqual(o.existingRows);
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Init o = (Init) other;
		return new Init(where, index, knownColumns.lub(o.knownColumns), existingRows.lub(existingRows));
	}

	@Override
	protected int compareToSameOperation(
			DataframeOperation o) {
		Init other = (Init) o;
		int cmp;
		if ((cmp = knownColumns.compareTo(other.knownColumns)) != 0)
			return cmp;
		return existingRows.compareTo(other.existingRows);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + Objects.hash(existingRows, knownColumns);
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Init other = (Init) obj;
		return Objects.equals(existingRows, other.existingRows) && Objects.equals(knownColumns, other.knownColumns);
	}

	@Override
	public String toString() {
		return "init(" + knownColumns + ", " + existingRows + ")";
	}
}
