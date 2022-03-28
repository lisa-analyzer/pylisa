package it.unive.pylisa.libraries;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.ReferenceType;
import it.unive.pylisa.cfg.type.PyClassType;

public class PyLibraryUnitType extends PyClassType {

	private final String libraryName;

	public PyLibraryUnitType(CompilationUnit library, CompilationUnit unit) {
		super(unit.getName(), unit);
		this.libraryName = library.getName();

		types.put(unit.getName(), this);
	}

	public String getLibraryName() {
		return libraryName;
	}

	public ReferenceType getReference() {
		return new ReferenceType(this);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((libraryName == null) ? 0 : libraryName.hashCode());
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
}
