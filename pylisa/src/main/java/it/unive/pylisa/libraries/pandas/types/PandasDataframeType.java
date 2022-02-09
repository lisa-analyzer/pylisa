package it.unive.pylisa.libraries.pandas.types;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collection;
import java.util.Set;

public class PandasDataframeType implements UnitType, PandasType {

	public static final PandasDataframeType INSTANCE = new PandasDataframeType();

	public static final ReferenceType REFERENCE = new ReferenceType(INSTANCE);

	private PandasDataframeType() {
	}

	@Override
	public String toString() {
		return "pandas.DataFrame";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PandasDataframeType;
	}

	@Override
	public int hashCode() {
		return PandasDataframeType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PandasDataframeType || other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(Type other) {
		return this;
	}

	@Override
	public Collection<Type> allInstances() {
		return Set.of(INSTANCE);
	}

	@Override
	public CompilationUnit getUnit() {
		return LibrarySpecificationProvider.getLibraryUnit(LibrarySpecificationProvider.PANDAS_DF);
	}
}
