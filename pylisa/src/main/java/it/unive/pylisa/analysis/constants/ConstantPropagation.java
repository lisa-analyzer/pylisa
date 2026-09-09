package it.unive.pylisa.analysis.constants;

import java.util.Objects;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;

/**
 * The lattice structure tracking a single constant value. The domain
 * operations (i.e. the evaluation of expressions) live in
 * {@link ConstantPropagationDomain}.
 */
public class ConstantPropagation
		implements
		BaseLattice<ConstantPropagation>,
		Comparable<ConstantPropagation> {

	static final ConstantPropagation TOP = new ConstantPropagation(null, true);
	static final ConstantPropagation BOTTOM = new ConstantPropagation(null, false);

	final Constant constant;

	private final boolean isTop;

	public ConstantPropagation() {
		this(null, true);
	}

	public ConstantPropagation(
			int value) {
		this(new Constant(Int32Type.INSTANCE, value, SyntheticLocation.INSTANCE));
	}

	public ConstantPropagation(
			Constant constant) {
		this(constant, false);
	}

	private ConstantPropagation(
			Constant constant,
			boolean isTop) {
		this.constant = constant;
		this.isTop = isTop;
	}

	public Object getConstant() {
		return constant.getValue();
	}

	public <T> boolean is(
			Class<T> type) {
		return type.isInstance(getConstant());
	}

	public <T> T as(
			Class<T> type) {
		return type.cast(getConstant());
	}

	@Override
	public String toString() {
		return representation().toString();
	}

	public StructuredRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		return new StringRepresentation(constant);
	}

	@Override
	public ConstantPropagation top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return BaseLattice.super.isTop() || (constant == null && isTop);
	}

	@Override
	public ConstantPropagation bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return BaseLattice.super.isBottom() || (constant == null && !isTop);
	}

	@Override
	public ConstantPropagation lubAux(
			ConstantPropagation other)
			throws SemanticException {
		return Objects.equals(constant, other.constant) ? this : top();
	}

	@Override
	public ConstantPropagation wideningAux(
			ConstantPropagation other)
			throws SemanticException {
		return lubAux(other);
	}

	@Override
	public boolean lessOrEqualAux(
			ConstantPropagation other)
			throws SemanticException {
		return Objects.equals(constant, other.constant);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((constant == null) ? 0 : constant.hashCode());
		result = prime * result + (isTop ? 1231 : 1237);
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ConstantPropagation other = (ConstantPropagation) obj;
		if (constant == null) {
			if (other.constant != null)
				return false;
		} else if (!constant.equals(other.constant))
			return false;
		if (isTop != other.isTop)
			return false;
		return true;
	}

	@Override
	public int compareTo(
			ConstantPropagation other) {
		if (isBottom() && !other.isBottom())
			return -1;
		else if (!isBottom() && other.isBottom())
			return 1;
		else if (isBottom())
			return 0;

		if (isTop() && !other.isTop())
			return 1;
		else if (!isTop() && other.isTop())
			return -1;
		else if (isTop())
			return 0;

		// not much we can do here..
		return Integer.compare(constant.hashCode(), other.constant.hashCode());
	}
}
