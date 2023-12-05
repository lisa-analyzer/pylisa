package it.unive.pylisa;

import it.unive.lisa.type.BooleanType;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.StringType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.cfg.type.PyBooleanType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyIntegralType;

public class PythonTypeSystem extends TypeSystem {

	@Override
	public BooleanType getBooleanType() {
		return (BooleanType) PyClassType.lookup(PyBooleanType.NAME);
	}

	@Override
	public StringType getStringType() {
		return it.unive.lisa.program.type.StringType.INSTANCE;
	}

	@Override
	public NumericType getIntegerType() {
		return (NumericType) PyClassType.lookup(PyIntegralType.NAME);
	}

	@Override
	public boolean canBeReferenced(
			Type type) {
		return type.isInMemoryType();
	}

}
