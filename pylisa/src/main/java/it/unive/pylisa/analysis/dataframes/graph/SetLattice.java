package it.unive.pylisa.analysis.dataframes.graph;

import java.util.Collections;
import java.util.Set;

public class SetLattice<E> extends it.unive.lisa.analysis.lattices.SetLattice<SetLattice<E>, E> {

	public SetLattice() {
		super(Collections.emptySet(), true);
	}

	public SetLattice(E element) {
		super(Collections.singleton(element), false);
	}

	public SetLattice(Set<E> elements, boolean isTop) {
		super(elements, isTop);
	}

	@Override
	public SetLattice<E> top() {
		return new SetLattice<>(Collections.emptySet(), true);
	}

	@Override
	public SetLattice<E> bottom() {
		return new SetLattice<>(Collections.emptySet(), false);
	}

	@Override
	protected SetLattice<E> mk(Set<E> set) {
		return new SetLattice<>(set, isTop);
	}
}
