package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.lisa.util.datastructures.graph.code.CodeNode;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.edge.DataframeEdge;

public abstract class DataframeOperation extends BaseLattice<DataframeOperation>
		implements Comparable<DataframeOperation>, CodeNode<DataframeForest, DataframeOperation, DataframeEdge> {

	public static final DataframeOperation TOP = new TopOperation();

	public static final DataframeOperation BOTTOM = new BottomOperation();

	protected final CodeLocation where;

	protected int offset;

	protected DataframeOperation(CodeLocation where) {
		this.where = where;
	}

	public CodeLocation getWhere() {
		return where;
	}

	@Override
	public final DataframeOperation top() {
		return TOP;
	}

	@Override
	public final DataframeOperation bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isTop() {
		return super.isTop() || TOP.equals(this);
	}

	@Override
	public boolean isBottom() {
		return super.isBottom() || BOTTOM.equals(this);
	}

	@Override
	protected DataframeOperation lubAux(DataframeOperation other) throws SemanticException {
		if (getClass() == other.getClass())
			return lubSameOperation(other);
		else if (where.equals(other.where))
			return new TopOperation(where);
		else
			return TOP;
	}

	@Override
	protected final DataframeOperation wideningAux(DataframeOperation other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeOperation other) throws SemanticException {
		return getClass() == other.getClass() ? lessOrEqualSameOperation(other) : false;
	}

	public CodeLocation loc(DataframeOperation other) {
		return where.equals(other.where) ? where : SyntheticLocation.INSTANCE;
	}

	protected abstract boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException;

	protected abstract DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException;

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((where == null) ? 0 : where.hashCode());
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
		DataframeOperation other = (DataframeOperation) obj;
		if (where == null) {
			if (other.where != null)
				return false;
		} else if (!where.equals(other.where))
			return false;
		return true;
	}

	@Override
	public abstract String toString();

	@Override
	public final int compareTo(DataframeOperation o) {
		int cmp;
		if ((cmp = where.compareTo(o.where)) != 0)
			return cmp;
		if ((cmp = getClass().getName().compareTo(o.getClass().getName())) != 0)
			return cmp;
		return compareToSameClassAndLocation(o);
	}

	protected abstract int compareToSameClassAndLocation(DataframeOperation o);

	@Override
	public <V> boolean accept(GraphVisitor<DataframeForest, DataframeOperation, DataframeEdge, V> visitor, V tool) {
		return visitor.visit(tool, null, this);
	}

	@Override
	public int setOffset(int offset) {
		return this.offset = offset;
	}

	@Override
	public int getOffset() {
		return offset;
	}
}
