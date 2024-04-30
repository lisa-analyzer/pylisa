package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;

import java.util.Collections;
import java.util.Set;

public class PyModuleType implements Type {

    public static final PyModuleType INSTANCE = new PyModuleType();

    protected PyModuleType() {}

    @Override
    public boolean canBeAssignedTo(Type other) {
        return other instanceof PyModuleType || other.isUntyped();
    }

    @Override
    public Type commonSupertype(Type other) {
        return (other instanceof PyModuleType ? this : Untyped.INSTANCE);
    }

    @Override
    public Set<Type> allInstances(TypeSystem types) {
        return Collections.singleton(this);
    }
}
