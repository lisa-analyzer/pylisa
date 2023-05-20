package it.unive.pylisa.analysis.dataframes.operations.selection;

import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.Names;

public class NumberSlice extends RowSelection<NumberSlice> {

	public static final NumberSlice TOP = new NumberSlice(
			new ConstantPropagation().top(),
			new ConstantPropagation().top(),
			new ConstantPropagation().top());
	public static final NumberSlice BOTTOM = new NumberSlice(
			new ConstantPropagation().bottom(),
			new ConstantPropagation().bottom(),
			new ConstantPropagation().bottom());

	private final ConstantPropagation beginIndex, endIndex, skip;

	public NumberSlice(int beginIndex, int endIndex, int skip) {
		this(new ConstantPropagation(beginIndex), new ConstantPropagation(endIndex), new ConstantPropagation(skip));
	}

	public NumberSlice(int beginIndex, int endIndex) {
		this(new ConstantPropagation(beginIndex), new ConstantPropagation(endIndex));
	}

	public NumberSlice(ConstantPropagation beginIndex, ConstantPropagation endIndex) {
		this(beginIndex, endIndex, new ConstantPropagation().bottom());
	}

	public NumberSlice(ConstantPropagation beginIndex, ConstantPropagation endIndex, ConstantPropagation skip) {
		this.beginIndex = beginIndex;
		this.endIndex = endIndex;
		this.skip = skip;
	}

	public ConstantPropagation getBeginIndex() {
		return beginIndex;
	}

	public ConstantPropagation getEndIndex() {
		return endIndex;
	}

	public ConstantPropagation getSkip() {
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
		String res = "[";
		if (!beginIndex.isBottom())
			res += beginIndex;
		res += ":";
		if (!endIndex.isBottom())
			res += endIndex;
		res += ":";
		if (!skip.isBottom())
			res += skip;
		res += "]";
		return res;
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
	public NumberSlice lubAux(NumberSlice other) throws SemanticException {
		return new NumberSlice(beginIndex.lub(other.beginIndex), endIndex.lub(other.endIndex), skip.lub(other.skip));
	}

	@Override
	public NumberSlice wideningAux(NumberSlice other) throws SemanticException {
		return new NumberSlice(beginIndex.widening(other.beginIndex), endIndex.widening(other.endIndex),
				skip.widening(other.skip));
	}

	@Override
	public boolean lessOrEqualAux(NumberSlice other) throws SemanticException {
		return beginIndex.lessOrEqual(other.beginIndex) && endIndex.lessOrEqual(other.endIndex)
				&& skip.lessOrEqual(other.skip);
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		NumberSlice other = (NumberSlice) o;
		int cmp = compare(beginIndex, other.beginIndex);
		if (cmp != 0)
			return cmp;
		cmp = compare(endIndex, other.endIndex);
		if (cmp != 0)
			return cmp;
		return compare(skip, other.skip);
	}

	private static int compare(ConstantPropagation first, ConstantPropagation second) {
		if (first.isBottom() && !second.isBottom())
			return -1;
		else if (!first.isBottom() && second.isBottom())
			return 1;
		else if (first.isBottom())
			return 0;

		if (first.isTop() && !second.isTop())
			return 1;
		else if (!first.isTop() && second.isTop())
			return -1;
		else if (first.isTop())
			return 0;

		return Integer.compare(first.as(Integer.class), second.as(Integer.class));
	}

	@Override
	public Names extractColumnNames() {
		return Names.BOTTOM;
	}
}
