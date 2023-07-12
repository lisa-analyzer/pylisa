package it.unive.pylisa.analysis.dataframes.operations.selection.rows;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;

public class AllRows extends RowSelection<AllRows> {

	public static final AllRows INSTANCE = new AllRows();

	private AllRows() {
	}

	@Override
	public AllRows lubSameClass(AllRows other) throws SemanticException {
		return this;
	}

	@Override
	protected AllRows wideningSameClass(AllRows other) throws SemanticException {
		return this;
	}

	@Override
	public boolean lessOrEqualSameClass(AllRows other) throws SemanticException {
		return true;
	}

	@Override
	public AllRows top() {
		return this;
	}

	@Override
	public AllRows bottom() {
		return this;
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		return 0;
	}

	@Override
	public Names extractColumnNames() throws SemanticException {
		return Names.BOTTOM;
	}

	@Override
	public String toString() {
		return "all";
	}
}
