package it.unive.pylisa.cfg;

import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

public class PyParameter extends Parameter {

	private String typeHint;

	private Expression dval;

	public PyParameter(
			CodeLocation location,
			String name) {
		super(location, name, Untyped.INSTANCE, null, new Annotations());
	}

	public PyParameter(
			CodeLocation location,
			String name,
			Type staticType) {
		super(location, name, staticType, null, new Annotations());
	}

	public PyParameter(
			CodeLocation location,
			String name,
			Type staticType,
			Annotations annotations) {
		super(location, name, staticType, null, annotations);
	}

	public String getTypeHint() {
		return typeHint;
	}

	public void setTypeHint(String typeHint) {
		this.typeHint = typeHint;
	}

	public void setDefaultValue(Expression defaultValue) {
		this.dval = defaultValue;
	}

	@Override
	public Expression getDefaultValue() {
		return this.dval != null ? this.dval : super.getDefaultValue();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((typeHint == null) ? 0 : typeHint.hashCode());
		result = prime * result + ((dval == null) ? 0 : dval.hashCode());
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		PyParameter other = (PyParameter) obj;
		if (typeHint == null) {
			if (other.typeHint != null)
				return false;
		} else if (!typeHint.equals(other.typeHint))
			return false;
		if (dval == null) {
			if (other.dval != null)
				return false;
		} else if (!dval.equals(other.dval))
			return false;
		return true;
	}

}
