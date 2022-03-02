package it.unive.pylisa.analysis.dataframes.transformation.operations;

import java.util.HashSet;
import java.util.Set;

public class DropColumns extends DataframeOperation {
    private Set<String> columns;

    public DropColumns(Set<String> columns) {
        this.columns = columns;
    }

    @Override
    protected boolean lessOrEqualSameOperation(DataframeOperation other) {
        DropColumns o = (DropColumns) other;
        if (this.columns == null)
            return o.columns == null;
        else if (o.columns == null)
            return true;
        return o.columns.equals(this.columns);
    }

    @Override
    protected DataframeOperation lubSameOperation(DataframeOperation other) {
        DropColumns o = (DropColumns) other;
        HashSet<String> newColumns = new HashSet<>();
        if (this.columns != null)
            newColumns.addAll(this.columns);
        if (o.columns != null)
            newColumns.addAll(o.columns);
        return new DropColumns(newColumns);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DropColumns other = (DropColumns) obj;
		if (columns == null) {
			if (other.columns != null)
				return false;
		} else if (!columns.equals(other.columns))
			return false;
		return true;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
		int result = 1;
		result = prime * result + ((columns == null) ? 0 : columns.hashCode());
		return result;
    }

    @Override
    public String toString() {
        return "drop_columns(" + String.join(",", columns) + ")";
    }

    
}
