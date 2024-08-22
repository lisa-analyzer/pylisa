package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.lattices.FunctionalLattice;
import it.unive.lisa.util.representation.MapRepresentation;
import it.unive.lisa.util.representation.SetRepresentation;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class CollectingMapLattice<K, V>
		extends
		FunctionalLattice<CollectingMapLattice<K, V>, K, SetLattice<V>> {

	public CollectingMapLattice(
			SetLattice<V> lattice) {
		super(lattice, null);
	}

	public CollectingMapLattice(
			SetLattice<V> lattice,
			Map<K, SetLattice<V>> function) {
		super(lattice, function);
	}

	public SetLattice<V> getLattice() {
		return lattice;
	}

	@Override
	public CollectingMapLattice<K, V> top() {
		return new CollectingMapLattice<>(lattice.top(), null);
	}

	@Override
	public CollectingMapLattice<K, V> bottom() {
		return new CollectingMapLattice<>(lattice.bottom(), null);
	}

	@Override
	public Map<K, SetLattice<V>> getMap() {
		if (function == null)
			return new HashMap<>();
		return super.getMap();
	}

	@Override
	public CollectingMapLattice<K, V> mk(
			SetLattice<V> lattice,
			Map<K, SetLattice<V>> function) {
		return new CollectingMapLattice<>(lattice, function);
	}

	@FunctionalInterface
	public interface Lifter<T> {
		T apply(
				T value)
				throws SemanticException;
	}

	public CollectingMapLattice<K, V> lift(
			Lifter<K> keyLifter,
			Lifter<SetLattice<V>> valueLifter)
			throws SemanticException {
		if (isBottom() || isTop() || function == null)
			return this;

		Map<K, SetLattice<V>> function = mkNewFunction(null, false);
		for (K id : getKeys()) {
			K liftedKey = keyLifter.apply(id);
			SetLattice<V> liftedValue = valueLifter.apply(getState(id));
			if (liftedKey != null && liftedValue != null)
				if (!function.containsKey(liftedKey))
					function.put(liftedKey, liftedValue);
				else
					function.put(liftedKey, liftedValue.lub(function.get(liftedKey)));
		}

		return mk(lattice, function);
	}

	public StructuredRepresentation representation(
			Function<V, StructuredRepresentation> valueMapper) {
		if (isTop())
			return Lattice.topRepresentation();

		if (isBottom())
			return Lattice.bottomRepresentation();

		if (function == null || function.isEmpty())
			return new StringRepresentation("");

		return new MapRepresentation(function, StringRepresentation::new,
				set -> new SetRepresentation(set.elements(), valueMapper));
	}

	@Override
	public SetLattice<V> stateOfUnknown(
			K key) {
		return lattice.bottom();
	}
}
