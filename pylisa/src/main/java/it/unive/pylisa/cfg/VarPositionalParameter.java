package it.unive.pylisa.cfg;

import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.type.Type;

public class VarPositionalParameter extends Parameter {

	public VarPositionalParameter(
			CodeLocation location,
			String name) {
		super(location, name);
	}

	public VarPositionalParameter(
			CodeLocation location,
			String name,
			Type staticType) {
		super(location, name, staticType);
	}

	public VarPositionalParameter(
			CodeLocation location,
			String name,
			Expression defaultValue) {
		super(location, name, defaultValue);
	}

	public VarPositionalParameter(
			CodeLocation location,
			String name,
			Type staticType,
			Expression defaultValue,
			Annotations annotations) {
		super(location, name, staticType, defaultValue, annotations);
	}
}