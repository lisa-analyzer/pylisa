package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import java.util.Objects;

public class Dataframe extends BaseNonRelationalValueDomain<Dataframe> {

    private final String file;
    private static final Dataframe top = new Dataframe(null);
    private static final Dataframe bottom = new Dataframe(null);
    private Dataframe(String s) {
        this.file=s;
    }

    @Override
    protected Dataframe lubAux(Dataframe other) throws SemanticException {
        if(this==other)
            return this;
        else return top;
    }

    @Override
    protected Dataframe wideningAux(Dataframe other) throws SemanticException {
        return lubAux(other);
    }

    @Override
    protected boolean lessOrEqualAux(Dataframe other) throws SemanticException {
        if(this==other)
            return true;
        else return false;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Dataframe dataframe = (Dataframe) o;
        return Objects.equals(file, dataframe.file);
    }

    @Override
    public int hashCode() {
        return Objects.hash(file);
    }


    @Override
    public DomainRepresentation representation() {
        if (isBottom())
            return Lattice.BOTTOM_REPR;
        else if (isTop())
            return Lattice.TOP_REPR;
        else return new StringRepresentation("File:"+file);
    }
    @Override
    public Dataframe top() {
        return top;
    }

    @Override
    public Dataframe bottom() {
        return bottom;
    }
}
