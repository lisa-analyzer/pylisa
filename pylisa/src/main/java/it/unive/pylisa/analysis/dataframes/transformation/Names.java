package it.unive.pylisa.analysis.dataframes.transformation;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.analysis.lattices.SetLattice;

public class Names extends SetLattice<Names, String> {

	private final boolean isTop;

	public Names() {
		this(true);
	}

	public Names(String element) {
		this(Collections.singleton(element), false);
	}

	public Names(Set<String> elements) {
		this(elements, false);
	}

	private Names(boolean isTop) {
		this(Collections.emptySet(), isTop);
	}

	private Names(Set<String> elements, boolean isTop) {
		super(elements);
		this.isTop = isTop;
	}

	@Override
	public Names top() {
		return new Names();
	}

	@Override
	public boolean isTop() {
		return elements.isEmpty() && isTop;
	}

	@Override
	public Names bottom() {
		return new Names(false);
	}

	@Override
	public boolean isBottom() {
		return elements.isEmpty() && !isTop;
	}

	@Override
	protected Names mk(Set<String> set) {
		return new Names(set);
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
		Names other = (Names) obj;
		if (isTop != other.isTop)
			return false;
		return true;
	}
}
