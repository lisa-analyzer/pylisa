package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.FunctionalLattice;
import it.unive.lisa.util.datastructures.trie.PatriciaTrieMap;
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
			PatriciaTrieMap<K, SetLattice<V>> function) {
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

	public Map<K, SetLattice<V>> getMap() {
		if (function == null)
			return new HashMap<>();
		return function.toHashMap();
	}

	/**
	 * Converts a plain {@link Map} into a {@link PatriciaTrieMap}, for
	 * callers that build/mutate a regular map (e.g. via {@link #getMap()}
	 * plus in-place {@code put}/{@code remove}) and then need to construct a
	 * {@link CollectingMapLattice} from the result.
	 */
	public static <K, V> PatriciaTrieMap<K, V> toTrieMap(
			Map<K, V> map) {
		PatriciaTrieMap<K, V> result = PatriciaTrieMap.empty();
		for (Map.Entry<K, V> e : map.entrySet())
			result = result.put(e.getKey(), e.getValue());
		return result;
	}

	@Override
	public CollectingMapLattice<K, V> mk(
			SetLattice<V> lattice,
			PatriciaTrieMap<K, SetLattice<V>> function) {
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

		PatriciaTrieMap<K, SetLattice<V>> function = mkNewFunction(null, false);
		for (K id : getKeys()) {
			K liftedKey = keyLifter.apply(id);
			SetLattice<V> liftedValue = valueLifter.apply(getState(id));
			if (liftedKey != null && liftedValue != null)
				if (!function.containsKey(liftedKey))
					function = function.put(liftedKey, liftedValue);
				else
					function = function.put(liftedKey, liftedValue.lub(function.get(liftedKey)));
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

		return new MapRepresentation(function.toHashMap(), StringRepresentation::new,
				set -> new SetRepresentation(set.elements(), valueMapper));
	}

	public CollectingMapLattice<K, V> setStack(
			SetLattice<V> stack) {
		return mk(stack, function == null ? null : mkNewFunction(function, false));
	}

	@Override
	public SetLattice<V> stateOfUnknown(
			K key) {
		return lattice.bottom();
	}
}
