package it.unive.pylisa.symbolic;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.Pair;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class DictConstant extends Constant {

	@SafeVarargs
	public DictConstant(CodeLocation location, Pair<Lattice<?>, Lattice<?>>... elements) {
		this(location, toMap(elements));
	}

	public DictConstant(CodeLocation location, Map<Lattice<?>, Lattice<?>> elements) {
		super(PyClassType.lookup(LibrarySpecificationProvider.DICT), elements, location);
	}

	public DictConstant(CodeLocation location, Map<Lattice<?>, Lattice<?>> elements,
			Pair<Lattice<?>, Lattice<?>> tail) {
		this(location, append(elements, tail));
	}

	private static Map<Lattice<?>, Lattice<?>> toMap(Pair<Lattice<?>, Lattice<?>>[] elements) {
		Map<Lattice<?>, Lattice<?>> result = new HashMap<>(elements.length);
		for (Pair<Lattice<?>, Lattice<?>> element : elements)
			result.put(element.getKey(), element.getValue());
		return result;
	}

	private static Map<Lattice<?>, Lattice<?>> append(Map<Lattice<?>, Lattice<?>> elements,
			Pair<Lattice<?>, Lattice<?>> tail) {
		Map<Lattice<?>, Lattice<?>> result = new HashMap<>(elements.size() + 1);
		result.putAll(elements);
		result.put(tail.getKey(), tail.getValue());
		return result;
	}

	@SuppressWarnings("unchecked")
	public Map<Lattice<?>, Lattice<?>> getList() {
		return (Map<Lattice<?>, Lattice<?>>) getValue();
	}
}
