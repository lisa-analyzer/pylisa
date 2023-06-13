package it.unive.pylisa.analysis.dataframes.operations.selection.columns;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;

public class AllColumns extends ColumnSelection<AllColumns> {

	public static final AllColumns INSTANCE = new AllColumns();

	private AllColumns() {
	}

	@Override
	public AllColumns lubSameClass(AllColumns other) throws SemanticException {
		return this;
	}

	@Override
	protected AllColumns wideningSameClass(AllColumns other) throws SemanticException {
		return this;
	}

	@Override
	public boolean lessOrEqualSameClass(AllColumns other) throws SemanticException {
		return true;
	}

	@Override
	public AllColumns top() {
		return this;
	}

	@Override
	public AllColumns bottom() {
		return this;
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		return 0;
	}

	@Override
	public Names extractColumnNames() throws SemanticException {
		return Names.TOP;
	}

	@Override
	public String toString() {
		return "all";
	}
}
