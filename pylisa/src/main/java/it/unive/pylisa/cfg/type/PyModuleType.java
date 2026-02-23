package it.unive.pylisa.cfg.type;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Unit;
import it.unive.lisa.type.InMemoryType;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import java.util.HashMap;
import java.util.Map;

import java.util.Set;

public class PyModuleType implements UnitType {

    private static final Map<String, PyModuleType> types = new HashMap<>();

    public static PyModuleType register(String name, CompilationUnit unit) {
        return types.computeIfAbsent(name, x -> new PyModuleType(name, unit));
    }

    public static PyModuleType lookup(String name) {
        PyModuleType t = types.get(name);
        if (t == null)
            throw new IllegalStateException("Module type '" + name + "' not registered");
        return t;
    }

    private final String name;
    private final CompilationUnit unit;

    private PyModuleType(String name, CompilationUnit unit) {
        this.name = name;
        this.unit = unit;
    }

    public CompilationUnit getUnit() {
        return unit;
    }

    public ReferenceType getReference() {
        return new ReferenceType(this);
    }

    @Override
    public boolean canBeAssignedTo(Type other) {
        // modules are only assignable to themselves
        return this.equals(other);
    }

    @Override
    public Type commonSupertype(Type other) {
        if (other.isNullType())
            return this;
        if (this.equals(other))
            return this;
        return Untyped.INSTANCE;
    }


    @Override
    public Set<Type> allInstances(TypeSystem types) {
        // Modules do not have subclasses
        return Set.of(this);
    }

    @Override
    public String toString() {
        return name;
    }

}