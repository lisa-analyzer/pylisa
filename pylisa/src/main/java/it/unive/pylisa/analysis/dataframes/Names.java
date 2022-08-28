package it.unive.pylisa.analysis.dataframes;

import java.util.Collections;
import java.util.Set;

import org.apache.commons.collections4.SetUtils;

import it.unive.lisa.analysis.lattices.SetLattice;
import it.unive.lisa.util.collections.CollectionsDiffBuilder;

public class Names extends SetLattice<Names, String> implements Comparable<Names> {

	public static final Names BOTTOM = new Names(false);
	public static final Names TOP = new Names();

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
		return TOP;
	}
	
	@Override
	public Names bottom() {
		return BOTTOM;
	}

	@Override
	protected Names mk(Set<String> set) {
		return new Names(set);
	}

	@Override
	public int compareTo(Names other) {
		int cmp;
		if ((cmp = Integer.compare(elements.size(), other.elements.size())) != 0)
			return cmp;

		CollectionsDiffBuilder<String> builder = new CollectionsDiffBuilder<>(String.class, elements, other.elements);
		builder.compute(String::compareTo);

		if (!builder.sameContent())
			// same size means that both have at least one element that is
			// different
			return builder.getOnlyFirst().iterator().next().compareTo(builder.getOnlySecond().iterator().next());
		return 0;
	}
	
	public Names intersection(Names other) {
		return new Names(SetUtils.intersection(elements, other.elements), false);
	}
	
	public Names symmetricDifference(Names other) {
		return new Names(SetUtils.disjunction(elements, other.elements), false);
	}
	
	public Names difference(Names other) {
		return new Names(SetUtils.difference(elements, other.elements), false);
	}
}
