package it.unive.pylisa.program;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.ProgramValidationException;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Collection;
import java.util.List;

public class ModuleUnit extends CompilationUnit {

	public ModuleUnit(
			CodeLocation location,
			Program program,
			String name) {
		this(location, program, name, true);
	}

	protected ModuleUnit(
			CodeLocation location,
			Program program,
			String name,
			boolean sealed) {
		super(location, program, name, sealed);
	}

	@Override
	public boolean addAncestor(
			CompilationUnit unit) {
		return false;
	}

	@Override
	public void addInstance(
			Unit unit)
			throws ProgramValidationException {

	}

	@Override
	public Collection<CompilationUnit> getImmediateAncestors() {
		return List.of();
	}

	@Override
	public boolean isInstanceOf(
			CompilationUnit unit) {
		return this == unit;
	}

	@Override
	public boolean canBeInstantiated() {
		return true;
	}
}
