package it.unive.pylisa.program;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.ProgramValidationException;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.Set;

public class UnknownModuleUnit extends ModuleUnit {

	private final Set<CompilationUnit> ancestors = new LinkedHashSet<>();

	public UnknownModuleUnit(
			CodeLocation location,
			Program program,
			String name) {
		super(location, program, name, false);
	}

	@Override
	public boolean addAncestor(
			CompilationUnit unit) {
		return ancestors.add(unit);
	}

	@Override
	public Collection<CompilationUnit> getImmediateAncestors() {
		return ancestors;
	}

	@Override
	public boolean isInstanceOf(
			CompilationUnit unit) {
		if (this == unit)
			return true;
		if (ancestors.contains(unit))
			return true;
		for (CompilationUnit ancestor : ancestors)
			if (ancestor.isInstanceOf(unit))
				return true;
		return false;
	}

	@Override
	public void addInstance(
			Unit unit)
			throws ProgramValidationException {
		// no instances tracked for unknown modules
	}
}
