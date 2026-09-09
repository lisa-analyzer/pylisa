package it.unive.pylisa;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.type.BooleanType;
import it.unive.lisa.type.CharacterType;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.StringType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;

public class PythonTypeSystem extends TypeSystem {

	@Override
	public BooleanType getBooleanType() {
		return BoolType.INSTANCE;
	}

	@Override
	public StringType getStringType() {
		return it.unive.lisa.program.type.StringType.INSTANCE;
	}

	@Override
	public NumericType getIntegerType() {
		return Int32Type.INSTANCE;
	}

	@Override
	public boolean canBeReferenced(
			Type type) {
		return type.isInMemoryType();
	}

	@Override
	public CharacterType getCharacterType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int distanceBetweenTypes(
			Type first,
			Type second) {
		// TODO Auto-generated method stub
		return 0;
	}

}
