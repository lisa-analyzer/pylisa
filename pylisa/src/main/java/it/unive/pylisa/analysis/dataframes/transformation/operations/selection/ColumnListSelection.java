package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

import java.util.Set;

public class ColumnListSelection extends Selection implements ColumnSelection {
    private Set<String> columns;
    private boolean allCols;

    public ColumnListSelection(Set<String> columns) {
        this.columns = columns;
        this.allCols = false;
    }

    public ColumnListSelection(Set<String> columns, boolean allCols) {
        this.columns = columns;
        this.allCols = allCols;
    }

    /**
     * @return Set<String> return the columns
     */
    public Set<String> getColumns() {
        return columns;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof ColumnListSelection))
            return false;
        
        ColumnListSelection o = (ColumnListSelection) obj;

        if (allCols && o.isAllCols())
            return true;

        return this.columns.equals(o.getColumns());
    }

    /**
     * @return boolean return if allCols set
     */
    public boolean isAllCols() {
        return allCols;
    }

}
