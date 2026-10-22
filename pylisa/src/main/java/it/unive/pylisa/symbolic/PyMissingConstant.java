package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.VoidType;

/**
 * The value of a parameter of a native function that was not passed (the
 * {@code missing} default of the library specifications), so that natives can
 * tell it apart from an explicit {@code None}, which is not a valid argument
 * for many builtins (e.g. {@code range(3, None)} raises {@code TypeError}). It
 * is never a value of the program.
 */
public class PyMissingConstant extends Constant {

	private static final Object MISSING = new Object();

	public PyMissingConstant(
			CodeLocation location) {
		super(VoidType.INSTANCE, MISSING, location);
	}

	@Override
	public int hashCode() {
		return super.hashCode() ^ getClass().getName().hashCode();
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "<missing>";
	}
}
