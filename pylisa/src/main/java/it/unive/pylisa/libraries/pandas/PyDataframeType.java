package it.unive.pylisa.libraries.pandas;

import java.util.Collection;
import java.util.Set;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PyDataframeType implements UnitType {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyDataframeType INSTANCE = new PyDataframeType();

	private PyDataframeType() {
	}

	@Override
	public String toString() {
		return "pandas.DataFrame";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PyDataframeType;
	}

	@Override
	public int hashCode() {
		return PyDataframeType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyDataframeType || other instanceof Untyped;
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
