package it.unive.pylisa.cfg.type;

import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;

import java.util.Set;

public class PyIntegerType implements NumericType {

    public static final PyIntegerType INSTANCE = new PyIntegerType();
    @Override
    public boolean is8Bits() {
        return false;
    }

    @Override
    public boolean is16Bits() {
        return false;
    }

    @Override
    public boolean is32Bits() {
        return false;
    }

    @Override
    public boolean is64Bits() {
        return false;
    }

    @Override
    public boolean isUnsigned() {
        return false;
    }

    @Override
    public boolean isIntegral() {
        return true;
    }


    @Override
    public boolean sameNumericTypes(NumericType other) {
        return other instanceof PyIntegerType;
    }

    @Override
    public NumericType supertype(NumericType other) {

        return this;
    }

    @Override
    public boolean canBeAssignedTo(Type other) {
        return other instanceof PyIntegerType;
    }

    @Override
    public Type commonSupertype(Type other) {
        return null;
    }

    @Override
    public Set<Type> allInstances(TypeSystem types) {
        return null;
    }
}
