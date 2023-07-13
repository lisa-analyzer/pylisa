package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.type.PyNotImplementedType;

public class PyNotImplementedConstant extends Constant {

	private static final Object NI_CONST = new Object();

	public PyNotImplementedConstant(CodeLocation location) {
		super(PyNotImplementedType.INSTANCE, NI_CONST, location);
	}

	@Override
	public int hashCode() {
		return super.hashCode() ^ getClass().getName().hashCode();
	}

	@Override
	public boolean equals(Object obj) {
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
		return "NotImplemented";
	}
}
