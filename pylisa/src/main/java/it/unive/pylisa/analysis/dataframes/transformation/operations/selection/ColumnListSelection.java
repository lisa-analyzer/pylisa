package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import java.util.Set;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.transformation.Names;

public class ColumnListSelection extends ColumnSelection<ColumnListSelection> {

	private static final ColumnListSelection TOP = new ColumnListSelection(new Names().top());
	private static final ColumnListSelection BOTTOM = new ColumnListSelection(new Names().bottom());

	private final Names columns;

	public ColumnListSelection(Set<String> columns) {
		this(new Names(columns));
	}

	public ColumnListSelection(Names columns) {
		this.columns = columns;
	}

	public ColumnListSelection(boolean allCols) {
		this(allCols ? new Names().top() : new Names().bottom());
	}

	public Names getColumns() {
		return columns;
	}

	public boolean isAllCols() {
		return columns.isTop();
	}

	@Override
	public ColumnListSelection top() {
		return TOP;
	}

	@Override
	public ColumnListSelection bottom() {
		return BOTTOM;
	}

	@Override
	protected ColumnListSelection lubAux(ColumnListSelection other) throws SemanticException {
		return new ColumnListSelection(columns.lub(other.columns));
	}

	@Override
	protected ColumnListSelection wideningAux(ColumnListSelection other) throws SemanticException {
		return new ColumnListSelection(columns.widening(other.columns));
	}

	@Override
	protected boolean lessOrEqualAux(ColumnListSelection other) throws SemanticException {
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
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ColumnListSelection other = (ColumnListSelection) obj;
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
}
