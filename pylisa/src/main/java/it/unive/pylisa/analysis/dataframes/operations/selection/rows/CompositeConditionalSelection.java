package it.unive.pylisa.analysis.dataframes.operations.selection.rows;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.pylisa.analysis.dataframes.Names;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;

public class CompositeConditionalSelection<L extends BooleanSelection<L>, R extends BooleanSelection<R>>
		extends BooleanSelection<CompositeConditionalSelection<L, R>> {

	public static enum Type {
		UNKNOWN,
		BOTTOM,
		AND,
		OR
	}

	private final Type type;
	private final L left;
	private final R right;

	public CompositeConditionalSelection(Type type, L bs1, R bs2) {
		this.type = type;
		this.left = bs1;
		this.right = bs2;
	}

	public Type getType() {
		return type;
	}

	public L getLeft() {
		return left;
	}

	public R getRight() {
		return right;
	}

	@Override
	public CompositeConditionalSelection<L, R> top() {
		return new CompositeConditionalSelection<>(Type.UNKNOWN, null, null);
	}

	@Override
	public boolean isTop() {
		return type == Type.UNKNOWN && left == null && right == null;
	}

	@Override
	public CompositeConditionalSelection<L, R> bottom() {
		return new CompositeConditionalSelection<>(Type.BOTTOM, null, null);
	}

	@Override
	public boolean isBottom() {
		return type == Type.BOTTOM && left == null && right == null;
	}

	@Override
	public CompositeConditionalSelection<L, R> lubSameClass(CompositeConditionalSelection<L, R> other)
			throws SemanticException {
		return type != other.type ? top()
				: new CompositeConditionalSelection<>(type, left.lub(other.left), right.lub(other.right));
	}

	@Override
	public CompositeConditionalSelection<L, R> wideningSameClass(CompositeConditionalSelection<L, R> other)
			throws SemanticException {
		return type != other.type ? top()
				: new CompositeConditionalSelection<>(type, left.widening(other.left), right.widening(other.right));
	}

	@Override
	public boolean lessOrEqualSameClass(CompositeConditionalSelection<L, R> other) throws SemanticException {
		return type != other.type ? false
				: left.lessOrEqual(other.left) && right.lessOrEqual(other.right);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((left == null) ? 0 : left.hashCode());
		result = prime * result + ((right == null) ? 0 : right.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
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
		CompositeConditionalSelection<?, ?> other = (CompositeConditionalSelection<?, ?>) obj;
		if (left == null) {
			if (other.left != null)
				return false;
		} else if (!left.equals(other.left))
			return false;
		if (right == null) {
			if (other.right != null)
				return false;
		} else if (!right.equals(other.right))
			return false;
		if (type != other.type)
			return false;
		return true;
	}

	@Override
	public String toString() {
		if (isTop())
			return Lattice.TOP_STRING;

		if (isBottom())
			return Lattice.BOTTOM_STRING;

		return left + " " + type.name() + " " + right;
	}

	@Override
	protected int compareToSameClass(Selection<?> o) {
		CompositeConditionalSelection<?, ?> other = (CompositeConditionalSelection<?, ?>) o;
		int cmp = type.compareTo(other.type);
		if (cmp != 0)
			return cmp;
		cmp = left.compareTo(other.left);
		if (cmp != 0)
			return cmp;
		return right.compareTo(other.right);
	}

	@Override
	public Names extractColumnNames() throws SemanticException {
		return left.extractColumnNames().lub(right.extractColumnNames());
	}
}
