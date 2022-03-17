package it.unive.pylisa.analysis.dataframes.transformation;

import java.util.Collections;
import java.util.Set;

import it.unive.lisa.analysis.lattices.SetLattice;

public class Names extends SetLattice<Names, String> {

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
		super(elements, isTop);
	}

	@Override
	public Names top() {
		return new Names();
	}

	@Override
	public Names bottom() {
		return new Names(false);
	}

	@Override
	protected Names mk(Set<String> set) {
		return new Names(set);
	}
}
