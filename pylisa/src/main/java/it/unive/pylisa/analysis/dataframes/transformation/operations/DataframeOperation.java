package it.unive.pylisa.analysis.dataframes.transformation.operations;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.lisa.util.datastructures.graph.Node;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeGraph;
import it.unive.pylisa.analysis.dataframes.transformation.graph.SimpleEdge;

public abstract class DataframeOperation extends BaseLattice<DataframeOperation>
		implements Node<DataframeOperation, SimpleEdge, DataframeGraph> {

	private static final DataframeOperation TOP = new TopOperation();

	private static final DataframeOperation BOTTOM = new BottomOpeartion();

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
		return getClass() == other.getClass() ? lubSameOperation(other) : TOP;
	}

	@Override
	protected final DataframeOperation wideningAux(DataframeOperation other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeOperation other) throws SemanticException {
		return getClass() == other.getClass() ? lessOrEqualSameOperation(other) : false;
	}

	protected abstract boolean lessOrEqualSameOperation(DataframeOperation other);

	protected abstract DataframeOperation lubSameOperation(DataframeOperation other);

	@Override
	public int setOffset(int offset) {
		return offset;
	}

	@Override
	public <V> boolean accept(GraphVisitor<DataframeGraph, DataframeOperation, SimpleEdge, V> visitor, V tool) {
		return visitor.visit(tool, null, this);
	}
}
