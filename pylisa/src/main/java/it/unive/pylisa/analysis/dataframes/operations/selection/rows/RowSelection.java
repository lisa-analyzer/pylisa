package it.unive.pylisa.analysis.dataframes.operations.selection.rows;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;

public abstract class RowSelection<R extends RowSelection<R>> extends Selection<R> {

	@SuppressWarnings("unchecked")
	@Override
	public final R lubAux(
			R other)
			throws SemanticException {
		if (getClass() != other.getClass())
			return (R) AllRows.INSTANCE;
		return lubSameClass(other);
	}

	@Override
	public final boolean lessOrEqualAux(
			R other)
			throws SemanticException {
		if (getClass() != other.getClass())
			return false;
		return lessOrEqualSameClass(other);
	}

	protected abstract boolean lessOrEqualSameClass(
			R other)
			throws SemanticException;

	protected abstract R lubSameClass(
			R other)
			throws SemanticException;
}
