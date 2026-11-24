package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.symbolic.value.Constant;

/**
 * The constant of a Python float literal. Python's floats are modeled with
 * 32-bits numbers, the value of this constant, while Python's value of the
 * literal is the 64-bits number closest to it, that is kept as the exact value
 * of this constant (the two differ for literals with more than 7 significant
 * digits).
 */
public class PyFloatConstant extends Constant {

	private final double exact;

	/**
	 * Builds the constant.
	 * 
	 * @param exact    Python's value of the literal
	 * @param location the location where the constant is defined
	 */
	public PyFloatConstant(
			double exact,
			CodeLocation location) {
		super(Float32Type.INSTANCE, (float) exact, location);
		this.exact = exact;
	}

	/**
	 * Yields Python's value of the literal, that is, the 64-bits number closest
	 * to it.
	 * 
	 * @return the value
	 */
	public double getExact() {
		return exact;
	}

	@Override
	public int hashCode() {
		return super.hashCode() ^ Double.hashCode(exact);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		return Double.doubleToLongBits(exact) == Double.doubleToLongBits(((PyFloatConstant) obj).exact);
	}
}
