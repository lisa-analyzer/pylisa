package it.unive.pylisa.libraries.pandas;

import java.util.Collection;
import java.util.Set;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PySeriesType implements UnitType {

	public static final PySeriesType INSTANCE = new PySeriesType();

	public static final ReferenceType REFERENCE = new ReferenceType(INSTANCE);

	private PySeriesType() {
	}

	@Override
	public String toString() {
		return "pandas.Series";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PySeriesType;
	}

	@Override
	public int hashCode() {
		return PySeriesType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PySeriesType || other instanceof Untyped;
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
