package it.unive.pylisa.cfg.type;

import java.util.Collection;
import java.util.Set;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PySliceType implements UnitType {

    public static final PySliceType INSTANCE = new PySliceType();

    private PySliceType() {
    }

    @Override
    public String toString() {
        return "slice";
    }

    @Override
    public boolean canBeAssignedTo(Type other) {
        return other instanceof PySliceType || other instanceof Untyped;
    }

    @Override
    public Type commonSupertype(Type other) {
        if (other == this)
            return this;
        return Untyped.INSTANCE;
    }

    @Override
    public Collection<Type> allInstances() {
        return Set.of(INSTANCE);
    }

    @Override
    public CompilationUnit getUnit() {
        return LibrarySpecificationProvider.getLibraryUnit(LibrarySpecificationProvider.SLICE);
    }
    
    
}
