package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.util.numeric.IntInterval;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.Objects;

/**
 * The product of a {@link ConstantPropagation} and of an {@link IntInterval}:
 * the interval over-approximates the integer values (where {@code False} and
 * {@code True} are {@code 0} and {@code 1}), and it is the top interval for the
 * values of any other type. The domain operations live in
 * {@link PyValueDomain}.
 */
public class PyValue implements BaseLattice<PyValue> {

	/**
	 * The top value.
	 */
	public static final PyValue TOP = new PyValue(ConstantPropagation.TOP, IntInterval.TOP);

	/**
	 * The bottom value.
	 */
	public static final PyValue BOTTOM = new PyValue(ConstantPropagation.BOTTOM, IntInterval.BOTTOM);

	private final ConstantPropagation constant;

	private final IntInterval interval;

	/**
	 * Builds the value.
	 *
	 * @param constant the constant
	 * @param interval the interval
	 */
	public PyValue(
			ConstantPropagation constant,
			IntInterval interval) {
		this.constant = constant;
		this.interval = interval;
	}

	/**
	 * Yields the constant.
	 *
	 * @return the constant
	 */
	public ConstantPropagation getConstant() {
		return constant;
	}

	/**
	 * Yields the interval.
	 *
	 * @return the interval
	 */
	public IntInterval getInterval() {
		return interval;
	}

	@Override
	public PyValue top() {
		return TOP;
	}

	@Override
	public PyValue bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isTop() {
		return constant.isTop() && interval.isTop();
	}

	@Override
	public boolean isBottom() {
		return constant.isBottom() || interval.isBottom();
	}

	@Override
	public PyValue lubAux(
			PyValue other)
			throws SemanticException {
		return new PyValue(constant.lub(other.constant), interval.lub(other.interval));
	}

	@Override
	public PyValue wideningAux(
			PyValue other)
			throws SemanticException {
		return new PyValue(constant.widening(other.constant), interval.widening(other.interval));
	}

	@Override
	public PyValue narrowingAux(
			PyValue other)
			throws SemanticException {
		return new PyValue(constant.isTop() ? other.constant : constant, interval.narrowing(other.interval));
	}

	@Override
	public PyValue glbAux(
			PyValue other)
			throws SemanticException {
		ConstantPropagation c = constant.isTop() ? other.constant
				: other.constant.isTop() || constant.equals(other.constant) ? constant : ConstantPropagation.BOTTOM;
		IntInterval i = interval.glb(other.interval);
		if (c.isBottom() || i.isBottom())
			return BOTTOM;
		return new PyValue(c, i);
	}

	@Override
	public boolean lessOrEqualAux(
			PyValue other)
			throws SemanticException {
		return constant.lessOrEqual(other.constant) && interval.lessOrEqual(other.interval);
	}

	@Override
	public StructuredRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		// the interval of a constant, or of a value that is not an integer,
		// adds nothing
		if (!constant.isTop() || interval.isTop())
			return constant.representation();
		return interval.representation();
	}

	@Override
	public String toString() {
		return representation().toString();
	}

	@Override
	public int hashCode() {
		return Objects.hash(constant, interval);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null || getClass() != obj.getClass())
			return false;
		PyValue other = (PyValue) obj;
		return constant.equals(other.constant) && interval.equals(other.interval);
	}

}
