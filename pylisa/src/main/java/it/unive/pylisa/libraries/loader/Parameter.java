package it.unive.pylisa.libraries.loader;

import java.util.Objects;

import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;

public class Parameter {

	private final String name;
	private final Type type;
	private final Value value;

	public enum ParameterType {
		STANDARD,
		VAR_ARGS,
		KW_ARGS,
		KW_ONLY
	}

	private final ParameterType parameterType;

	public Parameter(String name, Type type) {
		this(name, type, null, ParameterType.STANDARD);
	}

	public Parameter(String name, Type type, Value value) {
		this(name, type, value, ParameterType.STANDARD);
	}
	public Parameter(String name, Type type, Value value, ParameterType parameterType) {
		this.name = name;
		this.type = type;
		this.value = value;
		this.parameterType = parameterType;
	}

	public Parameter(String name, Type type, ParameterType parameterType) {
		this(name, type, null, parameterType);
	}

	public String getName() {
		return name;
	}

	public Type getType() {
		return type;
	}

	public Value getValue() {
		return value;
	}
	public ParameterType getParameterType() {
		return parameterType;
	}
	@Override
	public int hashCode() {
		return Objects.hash(name, type, value);
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Parameter other = (Parameter) obj;
		return Objects.equals(name, other.name) && Objects.equals(type, other.type)
				&& Objects.equals(value, other.value);
	}

	@Override
	public String toString() {
		return "Parameter [name=" + name + ", type=" + type + ", value=" + value + "]";
	}

	public it.unive.lisa.program.cfg.Parameter toLiSAParameter(CodeLocation location, CFG init) {
		Expression defValue = null;
		if (this.value != null) {
			defValue = this.value.toLiSAExpression(init);
		}
		switch (this.parameterType) {
			case VAR_ARGS:
				return new VarPositionalParameter(location, this.name, this.type.toLiSAType(), defValue, new Annotations());
			case KW_ARGS:
				return new VarKeywordParameter(location, this.name, this.type.toLiSAType(), defValue, new Annotations());
			case KW_ONLY:
				return new KeywordOnlyParameter(location, this.name, this.type.toLiSAType(), defValue, new Annotations());
			default:
				return new it.unive.lisa.program.cfg.Parameter(location, this.name, this.type.toLiSAType(),  defValue, new Annotations());
		}
	}
}
