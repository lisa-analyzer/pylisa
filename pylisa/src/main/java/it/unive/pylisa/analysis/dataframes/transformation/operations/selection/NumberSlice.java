package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.numeric.Interval;

public class NumberSlice extends RowSelection<NumberSlice> {

	private static final NumberSlice TOP = new NumberSlice(new Interval().top(), new Interval().top(),
			new Interval().top());
	private static final NumberSlice BOTTOM = new NumberSlice(new Interval().bottom(), new Interval().bottom(),
			new Interval().bottom());

	private final Interval beginIndex, endIndex, skip;

	public NumberSlice(int beginIndex, int endIndex, int skip) {
		this(new Interval(beginIndex, beginIndex), new Interval(endIndex, endIndex), new Interval(skip, skip));
	}

	public NumberSlice(int beginIndex, int endIndex) {
		this(new Interval(beginIndex, beginIndex), new Interval(endIndex, endIndex));
	}

	public NumberSlice(Interval beginIndex, Interval endIndex) {
		this(beginIndex, endIndex, new Interval().bottom());
	}

	public NumberSlice(Interval beginIndex, Interval endIndex, Interval skip) {
		this.beginIndex = beginIndex;
		this.endIndex = endIndex;
		this.skip = skip;
	}

	public Interval getBeginIndex() {
		return beginIndex;
	}

	public Interval getEndIndex() {
		return endIndex;
	}

	public Interval getSkip() {
		return skip;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((beginIndex == null) ? 0 : beginIndex.hashCode());
		result = prime * result + ((endIndex == null) ? 0 : endIndex.hashCode());
		result = prime * result + ((skip == null) ? 0 : skip.hashCode());
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
		NumberSlice other = (NumberSlice) obj;
		if (beginIndex == null) {
			if (other.beginIndex != null)
				return false;
		} else if (!beginIndex.equals(other.beginIndex))
			return false;
		if (endIndex == null) {
			if (other.endIndex != null)
				return false;
		} else if (!endIndex.equals(other.endIndex))
			return false;
		if (skip == null) {
			if (other.skip != null)
				return false;
		} else if (!skip.equals(other.skip))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "[" + beginIndex + ":" + endIndex + ":" + skip + "]";
	}

	@Override
	public NumberSlice top() {
		return TOP;
	}

	@Override
	public NumberSlice bottom() {
		return BOTTOM;
	}

	@Override
	protected NumberSlice lubAux(NumberSlice other) throws SemanticException {
		return new NumberSlice(beginIndex.lub(other.beginIndex), endIndex.lub(other.endIndex), skip.lub(other.skip));
	}

	@Override
	protected NumberSlice wideningAux(NumberSlice other) throws SemanticException {
		return new NumberSlice(beginIndex.widening(other.beginIndex), endIndex.widening(other.endIndex),
				skip.widening(other.skip));
	}

	@Override
	protected boolean lessOrEqualAux(NumberSlice other) throws SemanticException {
		return beginIndex.lessOrEqual(other.beginIndex) && endIndex.lessOrEqual(other.endIndex)
				&& skip.lessOrEqual(other.skip);
	}
}
