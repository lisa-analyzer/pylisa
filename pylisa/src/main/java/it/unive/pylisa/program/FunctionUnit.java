package it.unive.pylisa.program;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.ProgramValidationException;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import java.util.Collection;
import java.util.List;

public class FunctionUnit extends CompilationUnit {
	private CodeMember function;

	public CodeMember getFunction() {
		return function;
	}

	public FunctionUnit(
			CodeLocation location,
			Program program,
			String name,
			CodeMember function) {
		this(location, program, name, true);
		this.function = function;
	}

	public FunctionUnit(
			CodeLocation location,
			Program program,
			String name,
			boolean sealed) {
		super(location, program, name, sealed);
	}

	@Override
	public boolean addAncestor(
			it.unive.lisa.program.CompilationUnit unit) {
		return false;
	}

	@Override
	public void addInstance(
			Unit unit)
			throws ProgramValidationException {

	}

	@Override
	public Collection<it.unive.lisa.program.CompilationUnit> getImmediateAncestors() {
		return List.of();
	}

	@Override
	public boolean isInstanceOf(
			it.unive.lisa.program.CompilationUnit unit) {
		return this == unit;
	}

	@Override
	public boolean canBeInstantiated() {
		return false;
	}

	public void setFunction(
			CodeMember function) {
		this.function = function;
	}
}
