package it.unive.pylisa.analysis.dataframes.transformation.operations.selection;

public class NumberSlice extends Selection implements RowSelection {
    private int beginIndex, endIndex, skip;

    public NumberSlice(int beginIndex, int endIndex, int skip) {
        this.beginIndex = beginIndex;
        this.endIndex = endIndex;
        this.skip = skip;
    }

    public NumberSlice(int beginIndex, int endIndex) {
        this.beginIndex = beginIndex;
        this.endIndex = endIndex;
    }

    public int getBeginIndex() {
        return beginIndex;
    }

    public int getEndIndex() {
        return endIndex;
    }

    public int getSkip() {
        return skip;
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof NumberSlice))
            return false;

        NumberSlice o = (NumberSlice) obj;
        return beginIndex == o.getBeginIndex() 
                && endIndex == o.getEndIndex() 
                && skip == o.getSkip();
    }
}
