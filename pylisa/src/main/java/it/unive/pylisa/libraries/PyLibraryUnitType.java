package it.unive.pylisa.libraries;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;

public class PyLibraryUnitType extends PyClassType {

	private final String libraryName;

	private Integer hash = null;

	public PyLibraryUnitType(CompilationUnit library, CompilationUnit unit) {
		super(unit.getName(), unit);
		this.libraryName = library.getName();

		types.put(unit.getName(), this);
	}

	public String getLibraryName() {
		return libraryName;
	}

	@Override
	public int hashCode() {
		if (hash != null)
			return hash;
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((libraryName == null) ? 0 : libraryName.hashCode());
		hash = result;
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		PyLibraryUnitType other = (PyLibraryUnitType) obj;
		if (libraryName == null) {
			if (other.libraryName != null)
				return false;
		} else if (!libraryName.equals(other.libraryName))
			return false;
		return true;
	}

	public static boolean is(Type t, String lib, boolean includeReferences) {
		if (t instanceof PyLibraryUnitType)
			return ((PyLibraryUnitType) t).getLibraryName().equals(lib);
		else if (includeReferences && t instanceof ReferenceType)
			return ((ReferenceType) t).getInnerTypes().anyMatch(tt -> is(tt, lib, true));
		else
			return false;
	}
}
