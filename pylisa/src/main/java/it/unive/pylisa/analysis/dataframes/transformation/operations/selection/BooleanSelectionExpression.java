package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;

public class BooleanSelectionExpression<L extends BooleanSelection<L>, R extends BooleanSelection<R>>
		extends BooleanSelection<BooleanSelectionExpression<L, R>> {

	public static enum Type {
		UNKNOWN,
		BOTTOM,
		AND,
		OR
	}

	private final Type type;
	private final L left;
	private final R right;

	public BooleanSelectionExpression(Type type, L bs1, R bs2) {
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
	public BooleanSelectionExpression<L, R> top() {
		return new BooleanSelectionExpression<>(Type.UNKNOWN, null, null);
	}

	@Override
	public boolean isTop() {
		return type == Type.UNKNOWN && left == null && right == null;
	}

	@Override
	public BooleanSelectionExpression<L, R> bottom() {
		return new BooleanSelectionExpression<>(Type.BOTTOM, null, null);
	}

	@Override
	public boolean isBottom() {
		return type == Type.BOTTOM && left == null && right == null;
	}

	@Override
	protected BooleanSelectionExpression<L, R> lubAux(BooleanSelectionExpression<L, R> other) throws SemanticException {
		return type != other.type ? top()
				: new BooleanSelectionExpression<>(type, left.lub(other.left), right.lub(other.right));
	}

	@Override
	protected BooleanSelectionExpression<L, R> wideningAux(BooleanSelectionExpression<L, R> other)
			throws SemanticException {
		return type != other.type ? top()
				: new BooleanSelectionExpression<>(type, left.widening(other.left), right.widening(other.right));
	}

	@Override
	protected boolean lessOrEqualAux(BooleanSelectionExpression<L, R> other) throws SemanticException {
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
		BooleanSelectionExpression<?, ?> other = (BooleanSelectionExpression<?, ?>) obj;
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
}
