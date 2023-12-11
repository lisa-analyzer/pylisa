package it.unive.pylisa.analysis.dataframes.operations.selection;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.Names;

public abstract class Selection<S extends Selection<S>> implements BaseLattice<S>, Comparable<Selection<?>> {

	@SuppressWarnings("unchecked")
	public S lub(Selection<?> other) throws SemanticException {
		return lub((S) other);
	}

	@Override
	public final int compareTo(Selection<?> o) {
		int cmp;
		if ((cmp = getClass().getName().compareTo(o.getClass().getName())) != 0)
			return cmp;
		return compareToSameClass(o);
	}

	protected abstract int compareToSameClass(Selection<?> o);

	public abstract Names extractColumnNames() throws SemanticException;
}
