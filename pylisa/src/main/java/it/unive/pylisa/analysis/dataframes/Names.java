package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.lattices.SetLattice;
import java.util.Collections;
import java.util.Set;

public class Names extends SetLattice<Names, String> implements Comparable<Names> {

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

	@Override
	public int compareTo(Names knownColumns) {
		// TODO Auto-generated method stub
		return 0;
	}
}
