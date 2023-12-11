package it.unive.pylisa.cfg;

import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.type.Type;

public class KeywordOnlyParameter extends Parameter {

	public KeywordOnlyParameter(
			Parameter parameter) {
		super(parameter.getLocation(), parameter.getName(), parameter.getStaticType(), parameter.getDefaultValue(),
				parameter.getAnnotations());

	}

	public KeywordOnlyParameter(
			CodeLocation location,
			String name) {
		super(location, name);
	}

	public KeywordOnlyParameter(
			CodeLocation location,
			String name,
			Type staticType) {
		super(location, name, staticType);
	}

	public KeywordOnlyParameter(
			CodeLocation location,
			String name,
			Expression defaultValue) {
		super(location, name, defaultValue);
	}

	public KeywordOnlyParameter(
			CodeLocation location,
			String name,
			Type staticType,
			Expression defaultValue,
			Annotations annotations) {
		super(location, name, staticType, defaultValue, annotations);
	}
}