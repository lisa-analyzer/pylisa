package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.NumericType;

public class PyRealType extends PyClassType implements NumericType {

	public static final String NAME = "numbers.Real";

	public PyRealType(
			CompilationUnit unit) {
		super(NAME, unit);
		types.put(unit.getName(), this);
	}

	@Override
	public boolean is8Bits() {
		return false;
	}

	@Override
	public boolean is16Bits() {
		return false;
	}

	@Override
	public boolean is32Bits() {
		return false;
	}

	@Override
	public boolean is64Bits() {
		return false;
	}

	@Override
	public boolean isUnsigned() {
		return false;
	}

	@Override
	public boolean isIntegral() {
		return false;
	}
}
