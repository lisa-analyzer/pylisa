package it.unive.pylisa.symbolic.operators.dataframes;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.ColumnRangeSelection;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;

public class DataframeColumnSlice extends Constant {

	public DataframeColumnSlice(ColumnSlice slice, CodeLocation location) {
		super(PyClassType.lookup(LibrarySpecificationProvider.SLICE), slice, location);
	}

	public DataframeColumnSlice(ColumnRangeSelection start, ColumnRangeSelection end, RangeBound skip,
			CodeLocation location) {
		super(PyClassType.lookup(LibrarySpecificationProvider.SLICE), new ColumnSlice(start, end, skip), location);
	}

	public static class ColumnSlice {
		private ColumnRangeSelection start, end;
		private RangeBound skip;

		public ColumnSlice(ColumnRangeSelection start, ColumnRangeSelection end, RangeBound skip) {
			this.start = start;
			this.end = end;
			this.skip = skip;
		}

		public ColumnRangeSelection getStart() {
			return start;
		}

		public ColumnRangeSelection getEnd() {
			return end;
		}

		public RangeBound getSkip() {
			return skip;
		}

		@Override
		public boolean equals(Object obj) {
			if (!(obj instanceof ColumnSlice))
				return false;
			ColumnSlice o = (ColumnSlice) obj;
			if (this.start == null) {
				if (o.start != null)
					return false;
			} else if (!this.start.equals(o.start))
				return false;

			if (this.end == null) {
				if (o.end != null)
					return false;
			} else if (!this.end.equals(o.end))
				return false;

			if (this.skip == null) {
				if (o.skip != null)
					return false;
			} else if (!this.skip.equals(o.skip))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return start + ":" + end + ":" + skip;
		}
	}
}
