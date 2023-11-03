package it.unive.pylisa.analysis.dataframes.operations.selection.columns;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.NumberSlice;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;
import it.unive.pylisa.analysis.dataframes.operations.selection.SliceElement;

public class ColumnRangeSelection extends ColumnSelection<ColumnRangeSelection> implements SliceElement {

	public static final ColumnRangeSelection TOP = new ColumnRangeSelection(NumberSlice.TOP);
	public static final ColumnRangeSelection BOTTOM = new ColumnRangeSelection(NumberSlice.BOTTOM);

	private final NumberSlice columns;

	public ColumnRangeSelection(
			NumberSlice columns) {
		this.columns = columns;
	}

	public ColumnRangeSelection(
			int column) {
		this(new NumberSlice(column, column));
	}

	public ColumnRangeSelection(
			boolean allCols) {
		this(allCols ? NumberSlice.TOP : NumberSlice.BOTTOM);
	}

	public NumberSlice getColumns() {
		return columns;
	}

	public boolean isAllCols() {
		return columns.isTop();
	}

	@Override
	public ColumnRangeSelection top() {
		return TOP;
	}

	@Override
	public ColumnRangeSelection bottom() {
		return BOTTOM;
	}

	@Override
	public ColumnRangeSelection lubSameClass(
			ColumnRangeSelection other)
			throws SemanticException {
		return new ColumnRangeSelection(columns.lub(other.columns));
	}

	@Override
	public ColumnRangeSelection wideningSameClass(
			ColumnRangeSelection other)
			throws SemanticException {
		return new ColumnRangeSelection(columns.widening(other.columns));
	}

	@Override
	public boolean lessOrEqualSameClass(
			ColumnRangeSelection other)
			throws SemanticException {
		return columns.lessOrEqual(other.columns);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((columns == null) ? 0 : columns.hashCode());
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
		ColumnRangeSelection other = (ColumnRangeSelection) obj;
		if (columns == null) {
			if (other.columns != null)
				return false;
		} else if (!columns.equals(other.columns))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return columns.toString();
	}

	@Override
	protected int compareToSameClass(
			Selection<?> o) {
		ColumnRangeSelection other = (ColumnRangeSelection) o;
		return columns.compareTo(other.columns);
	}

	@Override
	public Names extractColumnNames() {
		return Names.BOTTOM;
	}

	@Override
	public ConstantPropagation getEndIndex() {
		return columns.getEndIndex();
	}

	@Override
	public ConstantPropagation getBeginIndex() {
		return columns.getBeginIndex();
	}
}
