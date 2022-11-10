package it.unive.pylisa;

import it.unive.lisa.type.BooleanType;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.StringType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.common.BoolType;
import it.unive.lisa.type.common.Int32Type;

public class PythonTypeSystem extends TypeSystem {

	@Override
	public BooleanType getBooleanType() {
		return BoolType.INSTANCE;
	}

	@Override
	public StringType getStringType() {
		return it.unive.lisa.type.common.StringType.INSTANCE;
	}

	@Override
	public NumericType getIntegerType() {
		return Int32Type.INSTANCE;
	}

	@Override
	public boolean canBeReferenced(Type type) {
		return type.isInMemoryType();
	}

}
