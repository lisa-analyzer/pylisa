package it.unive.pylisa.cfg.type;
import it.unive.lisa.type.*;

import java.util.*;

public class PyObjectType implements InMemoryType, PointerType {

    private final Type innerType;

    public PyObjectType(Type innerType) {
        this.innerType = innerType;
    }

    @Override
    public boolean canBeAssignedTo(Type other) {
        return true;
    }

    @Override
    public Type commonSupertype(Type other) {
        if (equals(other))
            return this;
        else if (other instanceof ReferenceType)
            return new PyObjectType(getInnerType().commonSupertype(((PyObjectType)other).getInnerType()));

        return Untyped.INSTANCE;
    }

    @Override
    public Set<Type> allInstances(TypeSystem types) {
        Set<Type> instances = new HashSet<>();
        for (Type inner : getInnerType().allInstances(types))
            instances.add(new PyObjectType(inner));
        instances.add(this);
        return instances;
    }

    @Override
    public Type getInnerType() {
        return innerType;
    }

    @Override
    public String toString() {
        return ">" + innerType;
    }
}
