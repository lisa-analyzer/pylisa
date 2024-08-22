package it.unive.pylisa.analysis.dataframes;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import org.apache.commons.collections4.SetUtils;
import org.apache.commons.collections4.SetUtils.SetView;

public class SetLattice<E> extends it.unive.lisa.analysis.lattices.SetLattice<SetLattice<E>, E> {

	public SetLattice() {
		super(Collections.emptySet(), true);
	}

	public SetLattice(
			E element) {
		super(Collections.singleton(element), false);
	}

	public SetLattice(
			Set<E> elements,
			boolean isTop) {
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
	public SetLattice<E> mk(
			Set<E> set) {
		return new SetLattice<>(set, isTop);
	}

	public boolean intersects(
			SetLattice<E> other) {
		return !SetUtils.intersection(elements, other.elements).isEmpty();
	}

	public SetLattice<E> remove(
			SetLattice<E> elements) {
		SetView<E> result = SetUtils.difference(this.elements, elements.elements);
		return new SetLattice<>(new HashSet<>(result), false);
	}

	public SetLattice<E> remove(
			Set<E> elements) {
		SetView<E> result = SetUtils.difference(this.elements, elements);
		return new SetLattice<>(new HashSet<>(result), false);
	}

	public SetLattice<E> replace(
			SetLattice<E> elements,
			SetLattice<E> targets) {
		SetView<E> result = SetUtils.union(
				SetUtils.difference(this.elements, elements.elements),
				targets.elements);

		return new SetLattice<>(new HashSet<>(result), false);
	}

	public SetLattice<E> replace(
			E element,
			E target) {
		HashSet<E> eles = new HashSet<>(elements);
		eles.remove(element);
		eles.add(target);
		return new SetLattice<>(eles, false);
	}
}
