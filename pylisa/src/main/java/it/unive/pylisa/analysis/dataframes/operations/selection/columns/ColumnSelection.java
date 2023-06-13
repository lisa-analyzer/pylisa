package it.unive.pylisa.analysis.dataframes.operations.selection.columns;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;

public abstract class ColumnSelection<C extends ColumnSelection<C>> extends Selection<C> {

	@SuppressWarnings("unchecked")
	@Override
	public final C lubAux(C other) throws SemanticException {
		if (getClass() != other.getClass())
			return (C) AllColumns.INSTANCE;
		return lubSameClass(other);
	}

	@SuppressWarnings("unchecked")
	@Override
	public final C wideningAux(C other) throws SemanticException {
		if (getClass() != other.getClass())
			return (C) AllColumns.INSTANCE;
		return wideningSameClass(other);
	}

	@Override
	public final boolean lessOrEqualAux(C other) throws SemanticException {
		if (getClass() != other.getClass())
			return false;
		return lessOrEqualSameClass(other);
	}

	protected abstract boolean lessOrEqualSameClass(C other) throws SemanticException;

	protected abstract C wideningSameClass(C other) throws SemanticException;

	protected abstract C lubSameClass(C other) throws SemanticException;
}
