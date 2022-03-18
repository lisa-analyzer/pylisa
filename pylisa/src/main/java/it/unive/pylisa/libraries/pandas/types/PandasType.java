package it.unive.pylisa.libraries.pandas.types;

import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;

public interface PandasType {
	public static boolean isPandasType(Type t, boolean includeReferences) {
		if (t instanceof PandasType)
			return true;

		if (!includeReferences)
			return false;

		return t instanceof ReferenceType
				&& ((ReferenceType) t).getInnerTypes().anyMatch(tt -> isPandasType(tt, includeReferences));
	}
}
