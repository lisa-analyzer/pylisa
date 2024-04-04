package it.unive.pylisa.cfg;

import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.type.Type;

public class PyParameter extends Parameter {

    private String typeHint;

    public PyParameter(CodeLocation location, String name) {
        super(location, name);
    }
    public PyParameter(CodeLocation location, String name, Type staticType) {
        super(location, name, staticType);
    }
    public PyParameter(CodeLocation location, String name, Type staticType, String typeHint) {
        super(location, name, staticType);
        this.typeHint = typeHint;
    }

    public PyParameter(CodeLocation location, String name, Expression defaultValue) {
        super(location, name, defaultValue);
    }

    public PyParameter(CodeLocation location, String name, Expression defaultValue, String typeHint) {
        super(location, name, defaultValue);
        this.typeHint = typeHint;
    }

    public PyParameter(CodeLocation location, String name, Type staticType, Expression defaultValue, Annotations annotations) {
        super(location, name, staticType, defaultValue, annotations);
    }
    public PyParameter(CodeLocation location, String name, Type staticType, Expression defaultValue, Annotations annotations, String typeHint) {
        super(location, name, staticType, defaultValue, annotations);
        this.typeHint = typeHint;
    }

    public String getTypeHint() {
        return typeHint;
    }
}
