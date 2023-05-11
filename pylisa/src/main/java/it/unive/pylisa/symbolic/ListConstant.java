package it.unive.pylisa.symbolic;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ListConstant extends Constant {

	public ListConstant(CodeLocation location, Lattice<?>... elements) {
		this(location, Arrays.asList(elements));
	}

	public ListConstant(CodeLocation location, List<Lattice<?>> elements) {
		super(PyClassType.lookup(LibrarySpecificationProvider.LIST), elements, location);
	}

	public ListConstant(CodeLocation location, List<Lattice<?>> elements, Lattice<?> tail) {
		this(location, append(elements, tail));
	}

	private static List<Lattice<?>> append(List<Lattice<?>> elements, Lattice<?> tail) {
		List<Lattice<?>> result = new ArrayList<>(elements.size() + 1);
		result.addAll(elements);
		result.add(tail);
		return result;
	}

	@SuppressWarnings("unchecked")
	public List<Lattice<?>> getList() {
		return (List<Lattice<?>>) getValue();
	}
}
