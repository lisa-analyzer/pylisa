package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

public abstract class ConstantFrom<Object> implements MappingFrom<Object> {
    Object o;

    public Object getValue() {
        return o;
    }

}
