package it.unive.pylisa.analysis;

import java.util.Arrays;
import java.util.Objects;
import java.util.Set;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;

public abstract class DataframeSingleTransformation extends BaseNonRelationalValueDomain<DataframeSingleTransformation> {

    private static final DataframeSingleTransformation top = new TopSingleTransformation(),
            bottom = new BottomSingleTransformation();

    @Override
    public final DataframeSingleTransformation top() {
        return top;
    }

    @Override
    public final DataframeSingleTransformation bottom() {
        return bottom;
    }

    @Override
    protected final DataframeSingleTransformation wideningAux(DataframeSingleTransformation other) throws SemanticException {
        return this.lubAux(other);
    }

    static class TopSingleTransformation extends DataframeSingleTransformation {
        @Override
        public DomainRepresentation representation() { return Lattice.TOP_REPR;}

        @Override
        protected DataframeSingleTransformation lubAux(DataframeSingleTransformation other) throws SemanticException {
            return this;
        }

        @Override
        protected boolean lessOrEqualAux(DataframeSingleTransformation other) throws SemanticException {
            return other instanceof TopSingleTransformation;
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof TopSingleTransformation;
        }

        @Override
        public int hashCode() {
            return 0;
        }
    }

    static class BottomSingleTransformation extends DataframeSingleTransformation {
        @Override
        public DomainRepresentation representation() {return Lattice.BOTTOM_REPR;}

        @Override
        protected DataframeSingleTransformation lubAux(DataframeSingleTransformation other) throws SemanticException {
            return other;
        }

        @Override
        protected boolean lessOrEqualAux(DataframeSingleTransformation other) throws SemanticException {
            return true;
        }

        @Override
        public boolean equals(Object obj) {
            return obj instanceof BottomSingleTransformation;
        }

        @Override
        public int hashCode() {
            return 0;
        }
    }

    static class Dataframe extends DataframeSingleTransformation {

        private final String file;
        public Dataframe(String s) {
            this.file=s;
        }

        @Override
        protected DataframeSingleTransformation lubAux(DataframeSingleTransformation other) throws SemanticException {
            if(other instanceof Dataframe && ((Dataframe) other).file.equals(this.file))
                return this;
            else return this.top();
        }

        @Override
        protected boolean lessOrEqualAux(DataframeSingleTransformation other) throws SemanticException {
            return false;
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
    }

    static class ProjectColumns extends DataframeSingleTransformation {
        private final Set<String> columns;

        ProjectColumns(Set<String> columns) {
            this.columns = columns;
        }

        @Override
        protected DataframeSingleTransformation lubAux(DataframeSingleTransformation other) throws SemanticException {
            if(this.equals(other))
                return this;
            else return this.top();
        }

        @Override
        protected boolean lessOrEqualAux(DataframeSingleTransformation other) throws SemanticException {
            return this.equals(other);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            ProjectColumns that = (ProjectColumns) o;
            return Objects.equals(columns, that.columns);
        }

        @Override
        public int hashCode() {
            return Objects.hash(columns);
        }

        @Override
        public DomainRepresentation representation() {
            return new StringRepresentation("ProjectColumns("+ Arrays.toString(columns.toArray())+")");
        }
    }
}
