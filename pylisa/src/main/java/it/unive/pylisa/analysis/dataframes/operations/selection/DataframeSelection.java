package it.unive.pylisa.analysis.dataframes.operations.selection;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.AllColumns;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.AllRows;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;

public class DataframeSelection<R extends RowSelection<R>, C extends ColumnSelection<C>>
		extends
		Selection<DataframeSelection<R, C>> {

	private final R rowSelection;
	private final C columnSelection;
	private final boolean isTop;

	public DataframeSelection(
			R rowSelection,
			C columnSelection) {
		this(rowSelection, columnSelection, false);
	}

	@SuppressWarnings("unchecked")
	public DataframeSelection(
			R rowSelection) {
		this(rowSelection, (C) AllColumns.INSTANCE, false);
	}

	@SuppressWarnings("unchecked")
	public DataframeSelection(
			C columnSelection) {
		this((R) AllRows.INSTANCE, columnSelection, false);
	}

	@SuppressWarnings("unchecked")
	public DataframeSelection(
			boolean isTop) {
		this((R) AllRows.INSTANCE, (C) AllColumns.INSTANCE, isTop);
	}

	public DataframeSelection(
			R rowSelection,
			C columnSelection,
			boolean isTop) {
		this.rowSelection = rowSelection;
		this.columnSelection = columnSelection;
		this.isTop = isTop;
	}

	public R getRowSelection() {
		return rowSelection;
	}

	public C getColumnSelection() {
		return columnSelection;
	}

	public boolean isAllRows() {
		return rowSelection == null;
	}

	public boolean isAllColumns() {
		return columnSelection == null;
	}

	@Override
	public DataframeSelection<R, C> top() {
		return new DataframeSelection<>(true);
	}

	@Override
	public boolean isTop() {
		return rowSelection == null && columnSelection == null && isTop;
	}

	@Override
	public DataframeSelection<R, C> bottom() {
		return new DataframeSelection<>(false);
	}

	@Override
	public boolean isBottom() {
		return rowSelection == null && columnSelection == null && !isTop;
	}

	@Override
	public DataframeSelection<R, C> lubAux(
			DataframeSelection<R, C> other)
			throws SemanticException {
		return new DataframeSelection<>(nullSafe(rowSelection, other.rowSelection, Lattice::lub),
				nullSafe(columnSelection, other.columnSelection, Lattice::lub));
	}

	@Override
	public DataframeSelection<R, C> wideningAux(
			DataframeSelection<R, C> other)
			throws SemanticException {
		return new DataframeSelection<>(nullSafe(rowSelection, other.rowSelection, Lattice::widening),
				nullSafe(columnSelection, other.columnSelection, Lattice::widening));
	}

	@Override
	public boolean lessOrEqualAux(
			DataframeSelection<R, C> other)
			throws SemanticException {
		if (nullSafe(rowSelection, other.rowSelection, Lattice::lub) == null)
			return false;

		if (nullSafe(columnSelection, other.columnSelection, Lattice::lub) == null)
			return false;

		return rowSelection.lessOrEqual(other.rowSelection) && columnSelection.lessOrEqual(other.columnSelection);
	}

	@FunctionalInterface
	public interface BiSemanticFunction<T, U, R> {
		R apply(
				T t,
				U u)
				throws SemanticException;
	}

	private static <L extends Lattice<L>> L nullSafe(
			L left,
			L right,
			BiSemanticFunction<L, L, L> func)
			throws SemanticException {
		if (left == null)
			return null;

		if (right == null)
			return null;

		return func.apply(left, right);
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
		DataframeSelection<?, ?> other = (DataframeSelection<?, ?>) obj;
		if (columnSelection == null) {
			if (other.columnSelection != null)
				return false;
		} else if (!columnSelection.equals(other.columnSelection))
			return false;
		if (rowSelection == null) {
			if (other.rowSelection != null)
				return false;
		} else if (!rowSelection.equals(other.rowSelection))
			return false;
		return true;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((columnSelection == null) ? 0 : columnSelection.hashCode());
		result = prime * result + ((rowSelection == null) ? 0 : rowSelection.hashCode());
		return result;
	}

	@Override
	public String toString() {
		return "[rows:" + rowSelection + ", cols:" + columnSelection + "]";
	}

	@Override
	protected int compareToSameClass(
			Selection<?> o) {
		DataframeSelection<?, ?> other = (DataframeSelection<?, ?>) o;
		int cmp = Boolean.compare(isTop, other.isTop);
		if (cmp != 0)
			return cmp;
		cmp = columnSelection.compareTo(other.columnSelection);
		if (cmp != 0)
			return cmp;
		return rowSelection.compareTo(other.rowSelection);
	}

	@Override
	public Names extractColumnNames() throws SemanticException {
		// we do not extract column names from row filters intentionally, as
		// they are handled ad-hoc
		return columnSelection == null ? Names.TOP : columnSelection.extractColumnNames();
	}
}
