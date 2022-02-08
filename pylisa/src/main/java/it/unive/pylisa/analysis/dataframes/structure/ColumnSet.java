package it.unive.pylisa.analysis.dataframes.structure;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.lattices.InverseSetLattice;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.SetRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;

public class ColumnSet extends InverseSetLattice<ColumnSet, String> {

	private final boolean isTop;

	public ColumnSet() {
		this(null, true);
	}

	public ColumnSet(ColumnSet other) {
		this(other.elements == null ? null : Set.copyOf(other.elements), other.isTop);
	}

	public ColumnSet(String element) {
		this(Collections.singleton(element), false);
	}

	public ColumnSet(Set<String> elements) {
		this(elements, false);
	}

	private ColumnSet(Set<String> elements, boolean isTop) {
		super(elements);
		this.isTop = isTop;
	}

	@Override
	public ColumnSet top() {
		return new ColumnSet();
	}

	@Override
	public boolean isTop() {
		return elements == null && isTop;
	}

	@Override
	public ColumnSet bottom() {
		return new ColumnSet(null, false);
	}

	@Override
	public boolean isBottom() {
		return elements == null && !isTop;
	}

	@Override
	protected ColumnSet mk(Set<String> set) {
		return new ColumnSet(set);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + (isTop ? 1231 : 1237);
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
		ColumnSet other = (ColumnSet) obj;
		if (isTop != other.isTop)
			return false;
		return true;
	}

	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;

		if (isBottom())
			return Lattice.BOTTOM_REPR;

		return new SetRepresentation(elements, StringRepresentation::new);
	}
}
