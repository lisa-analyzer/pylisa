package it.unive.pylisa.symbolic.operators.dataframes.aux;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.analysis.dataframes.SetLattice;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.operations.selection.SliceElement;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;

public class DataframeColumnSlice extends Constant {

	public DataframeColumnSlice(ColumnSlice slice, CodeLocation location) {
		super(PyClassType.lookup(LibrarySpecificationProvider.SLICE), slice, location);
	}

	public static class ColumnSlice {
		private SliceElement start, end;
		private SetLattice<DataframeOperation> startNodes, endNodes;
		private RangeBound skip;

		public ColumnSlice(
				SliceElement start,
				SliceElement end,
				RangeBound skip,
				SetLattice<DataframeOperation> startNodes,
				SetLattice<DataframeOperation> endNodes) {
			this.start = start;
			this.end = end;
			this.skip = skip;
			this.startNodes = startNodes;
			this.endNodes = endNodes;
		}

		public SliceElement getStart() {
			return start;
		}

		public SetLattice<DataframeOperation> getStartNodes() {
			return startNodes;
		}

		public SliceElement getEnd() {
			return end;
		}

		public SetLattice<DataframeOperation> getEndNodes() {
			return endNodes;
		}

		public RangeBound getSkip() {
			return skip;
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + ((end == null) ? 0 : end.hashCode());
			result = prime * result + ((endNodes == null) ? 0 : endNodes.hashCode());
			result = prime * result + ((skip == null) ? 0 : skip.hashCode());
			result = prime * result + ((start == null) ? 0 : start.hashCode());
			result = prime * result + ((startNodes == null) ? 0 : startNodes.hashCode());
			return result;
		}

		@Override
		public boolean equals(Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			ColumnSlice other = (ColumnSlice) obj;
			if (end == null) {
				if (other.end != null)
					return false;
			} else if (!end.equals(other.end))
				return false;
			if (endNodes == null) {
				if (other.endNodes != null)
					return false;
			} else if (!endNodes.equals(other.endNodes))
				return false;
			if (skip == null) {
				if (other.skip != null)
					return false;
			} else if (!skip.equals(other.skip))
				return false;
			if (start == null) {
				if (other.start != null)
					return false;
			} else if (!start.equals(other.start))
				return false;
			if (startNodes == null) {
				if (other.startNodes != null)
					return false;
			} else if (!startNodes.equals(other.startNodes))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return start + ":" + end + ":" + skip;
		}
	}
}
