package it.unive.pylisa.analysis.dataframes.operations.selection;

import it.unive.lisa.analysis.SemanticException;

public class RowFilter<B extends BooleanSelection<B>> extends RowSelection<RowFilter<B>> {

	private static final RowFilter<?> TOP = new RowFilter<>(AtomicBooleanSelection.TOP);
	private static final RowFilter<?> BOTTOM = new RowFilter<>(AtomicBooleanSelection.BOTTOM);

	private B selection;

	public RowFilter(B selection) {
		this.selection = selection;
	}

	public B getSelection() {
		return selection;
	}

	@Override
	@SuppressWarnings("unchecked")
	public RowFilter<B> top() {
		return (RowFilter<B>) TOP;
	}

	@Override
	@SuppressWarnings("unchecked")
	public RowFilter<B> bottom() {
		return (RowFilter<B>) BOTTOM;
	}

	@Override
	protected RowFilter<B> wideningAux(RowFilter<B> other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(RowFilter<B> other) throws SemanticException {
		if (this.equals(other))
			return true;
		if (!this.selection.getClass().equals(other.selection.getClass()))
			return false;
		return selection.lessOrEqual(other.selection);
	}

	@Override
	@SuppressWarnings("unchecked")
	protected RowFilter<B> lubAux(RowFilter<B> other) throws SemanticException {
		if (!this.selection.getClass().equals(other.selection.getClass())) {
			return (RowFilter<B>) TOP;
		}
		return new RowFilter<>(selection.lub(other.selection));
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
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
		RowFilter<?> other = (RowFilter<?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return selection.toString();
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		RowFilter<?> other = (RowFilter<?>) o;
		return selection.compareTo(other.selection);
	}
}
