package it.unive.pylisa.symbolic;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class SetConstant extends Constant {

  public SetConstant(CodeLocation location, Lattice<?>... elements) {
    this(location, new HashSet<Lattice<?>>(Arrays.asList(elements)));
  }

  public SetConstant(CodeLocation location, Set<Lattice<?>> elements) {
    super(PyClassType.lookup(LibrarySpecificationProvider.SET), elements, location);
  }

  public SetConstant(CodeLocation location, Set<Lattice<?>> elements, Lattice<?> tail) {
    this(location, append(elements, tail));
  }

  private static Set<Lattice<?>> append(Set<Lattice<?>> elements, Lattice<?> tail) {
    Set<Lattice<?>> result = new HashSet<>();
    result.addAll(elements);
    result.add(tail);
    return result;
  }

  @SuppressWarnings("unchecked")
  public List<Lattice<?>> getList() {
    return (List<Lattice<?>>) getValue();
  }
}