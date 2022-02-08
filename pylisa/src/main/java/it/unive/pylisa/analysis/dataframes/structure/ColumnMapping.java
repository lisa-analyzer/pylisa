package it.unive.pylisa.analysis.dataframes.structure;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.lattices.FunctionalLattice;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.MapRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;

public class ColumnMapping extends FunctionalLattice<ColumnMapping, Integer, ColumnSet> {

	public ColumnMapping() {
		super(new ColumnSet());
	}

	public ColumnMapping(ColumnMapping other) {
		this(other.lattice, copy(other.function));
	}

	private static Map<Integer, ColumnSet> copy(Map<Integer, ColumnSet> function) {
		Map<Integer, ColumnSet> result = new HashMap<>(function.size());
		for (Entry<Integer, ColumnSet> entry : function.entrySet())
			result.put(entry.getKey(), new ColumnSet(entry.getValue()));
		return result;
	}

	private ColumnMapping(ColumnSet lattice, Map<Integer, ColumnSet> function) {
		super(lattice, function);
	}

	@Override
	public ColumnMapping top() {
		return new ColumnMapping(lattice.top(), null);
	}

	@Override
	public boolean isTop() {
		return lattice.isTop() && function == null;
	}

	@Override
	public ColumnMapping bottom() {
		return new ColumnMapping(lattice.bottom(), null);
	}

	@Override
	public boolean isBottom() {
		return lattice.isBottom() && function == null;
	}

	@Override
	protected ColumnMapping mk(ColumnSet lattice, Map<Integer, ColumnSet> function) {
		return new ColumnMapping(lattice, function);
	}

	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;

		if (isBottom())
			return Lattice.BOTTOM_REPR;

		return new MapRepresentation(function, StringRepresentation::new, ColumnSet::representation);
	}
}
