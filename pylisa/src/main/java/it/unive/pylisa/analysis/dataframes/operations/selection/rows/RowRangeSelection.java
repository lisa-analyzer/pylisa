package it.unive.pylisa.analysis.dataframes.operations.selection.rows;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.NumberSlice;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;
import it.unive.pylisa.analysis.dataframes.operations.selection.SliceElement;

public class RowRangeSelection extends RowSelection<RowRangeSelection> implements SliceElement {

	public static final RowRangeSelection TOP = new RowRangeSelection(NumberSlice.TOP);
	public static final RowRangeSelection BOTTOM = new RowRangeSelection(NumberSlice.BOTTOM);

	private final NumberSlice rows;

	public RowRangeSelection(NumberSlice rows) {
		this.rows = rows;
	}

	public RowRangeSelection(int row) {
		this(new NumberSlice(row, row));
	}

	public RowRangeSelection(boolean allRows) {
		this(allRows ? NumberSlice.TOP : NumberSlice.BOTTOM);
	}

	public NumberSlice getRows() {
		return rows;
	}

	public boolean isAllRows() {
		return rows.isTop();
	}

	@Override
	public RowRangeSelection top() {
		return TOP;
	}

	@Override
	public RowRangeSelection bottom() {
		return BOTTOM;
	}

	@Override
	public RowRangeSelection lubSameClass(RowRangeSelection other) throws SemanticException {
		return new RowRangeSelection(rows.lub(other.rows));
	}

	@Override
	public RowRangeSelection wideningSameClass(RowRangeSelection other) throws SemanticException {
		return new RowRangeSelection(rows.widening(other.rows));
	}

	@Override
	public boolean lessOrEqualSameClass(RowRangeSelection other) throws SemanticException {
		return rows.lessOrEqual(other.rows);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((rows == null) ? 0 : rows.hashCode());
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
		RowRangeSelection other = (RowRangeSelection) obj;
		if (rows == null) {
			if (other.rows != null)
				return false;
		} else if (!rows.equals(other.rows))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return rows.toString();
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		RowRangeSelection other = (RowRangeSelection) o;
		return rows.compareTo(other.rows);
	}

	@Override
	public Names extractColumnNames() {
		return Names.BOTTOM;
	}

	@Override
	public ConstantPropagation getEndIndex() {
		return rows.getEndIndex();
	}

	@Override
	public ConstantPropagation getBeginIndex() {
		return rows.getBeginIndex();
	}
}
