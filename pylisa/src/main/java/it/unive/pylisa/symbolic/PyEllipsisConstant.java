package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.type.PyClassType;

public class PyEllipsisConstant extends Constant {

	private static final Object ELLIPSIS_CONST = new Object();

	public PyEllipsisConstant(
			CodeLocation location) {
		super(PyClassType.lookup("builtins.ellipsis"), ELLIPSIS_CONST, location);
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
		return "...";
	}
}
