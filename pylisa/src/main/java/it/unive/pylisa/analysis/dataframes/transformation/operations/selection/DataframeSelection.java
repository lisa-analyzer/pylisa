package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

public class DataframeSelection {
    private RowSelection rowSelection;
    private ColumnSelection columnSelection;

    private boolean allRows;
    private boolean allColumns;

    public DataframeSelection(RowSelection rowSelection, ColumnSelection columnSelection) {
        this.rowSelection = rowSelection;
        this.columnSelection = columnSelection;

        this.allRows = false;
        this.allColumns = false;
    }

    public DataframeSelection(RowSelection rowSelection) {
        this.rowSelection = rowSelection;

        this.allRows = false;
        this.allColumns = true;
    }

    public DataframeSelection(ColumnSelection columnSelection) {
        this.columnSelection = columnSelection;

        this.allRows = true;
        this.allColumns = false;
    }

    /**
     * @return RowSelection return the rowSelection
     */
    public RowSelection getRowSelection() {
        return rowSelection;
    }

    /**
     * @return ColumnSelection return the columnSelection
     */
    public ColumnSelection getColumnSelection() {
        return columnSelection;
    }

    /**
     * @return boolean return the allRows
     */
    public boolean isAllRows() {
        return allRows;
    }

    /**
     * @return boolean return the allColumns
     */
    public boolean isAllColumns() {
        return allColumns;
    }

    private boolean sameColumns(DataframeSelection other) {
        if (allColumns && other.allColumns)
            return true;
        return columnSelection.equals(other.columnSelection);
    }

    private boolean sameRows(DataframeSelection other) {
        if (allRows && other.allRows)
            return true;
        return rowSelection.equals(other.rowSelection);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof DataframeSelection))
            return false;
        
        DataframeSelection o = (DataframeSelection) obj;

        return sameColumns(o) && sameRows(o);
    }

}
