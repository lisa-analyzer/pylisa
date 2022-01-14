package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.symbolic.LibraryIdentifier;
import java.util.Objects;

public class LibraryDomain extends BaseNonRelationalValueDomain<LibraryDomain> {

	private final String library_name;
	private static final LibraryDomain top = new LibraryDomain(null);
	private static final LibraryDomain bottom = new LibraryDomain(null);

	public LibraryDomain(String s) {
		this.library_name = s;
	}

	@Override
	protected LibraryDomain evalNonNullConstant(Constant constant, ProgramPoint pp) throws SemanticException {
		if (constant instanceof LibraryIdentifier)
			return new LibraryDomain(((LibraryIdentifier) constant).getValue().toString());
		else
			return super.evalNonNullConstant(constant, pp);
	}

	@Override
	protected LibraryDomain lubAux(LibraryDomain other) throws SemanticException {
		if (this == other)
			return this;
		else
			return top;
	}

	@Override
	protected LibraryDomain wideningAux(LibraryDomain other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(LibraryDomain other) throws SemanticException {
		if (this == other)
			return true;
		else
			return false;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o)
			return true;
		if (o == null || getClass() != o.getClass())
			return false;
		LibraryDomain dataframe = (LibraryDomain) o;
		return Objects.equals(library_name, dataframe.library_name);
	}

	@Override
	public int hashCode() {
		return Objects.hash(library_name);
	}

	@Override
	public DomainRepresentation representation() {
		if (isBottom())
			return Lattice.BOTTOM_REPR;
		else if (isTop())
			return Lattice.TOP_REPR;
		else
			return new StringRepresentation("Library:" + library_name);
	}

	@Override
	public LibraryDomain top() {
		return top;
	}

	@Override
	public LibraryDomain bottom() {
		return bottom;
	}
}
