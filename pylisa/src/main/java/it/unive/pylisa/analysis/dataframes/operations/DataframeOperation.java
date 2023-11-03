package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.lisa.util.datastructures.graph.code.CodeNode;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import it.unive.pylisa.analysis.dataframes.DataframeForest;
import it.unive.pylisa.analysis.dataframes.edge.DataframeEdge;
import java.util.Objects;

public abstract class DataframeOperation
		implements
		BaseLattice<DataframeOperation>,
		Comparable<DataframeOperation>,
		CodeNode<DataframeForest, DataframeOperation, DataframeEdge> {

	public static final DataframeOperation TOP = new TopOperation();

	public static final DataframeOperation BOTTOM = new BottomOperation();

	protected final CodeLocation where;

	protected final int index;

	/**
	 * Internal field for the graph class, not involved in equality/semantic
	 * operations
	 */
	protected int offset;

	protected DataframeOperation(
			CodeLocation where,
			int index) {
		this.where = where;
		this.index = index;
	}

	public CodeLocation getWhere() {
		return where;
	}

	public int getIndex() {
		return index;
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
		return BaseLattice.super.isTop() || TOP == this || TOP.equals(this);
	}

	@Override
	public boolean isBottom() {
		return BaseLattice.super.isBottom() || BOTTOM == this || BOTTOM.equals(this);
	}

	public final boolean similar(
			DataframeOperation other) {
		return getClass() == other.getClass() && where.equals(other.where) && index == other.index;
	}

	@Override
	public final boolean lessOrEqualAux(
			DataframeOperation other)
			throws SemanticException {
		return similar(other) ? lessOrEqualSameOperation(other) : false;
	}

	@Override
	public final DataframeOperation lubAux(
			DataframeOperation other)
			throws SemanticException {
		if (similar(other))
			return lubSameOperation(other);
		else
			return TOP;
	}

	@Override
	public final DataframeOperation wideningAux(
			DataframeOperation other)
			throws SemanticException {
		if (similar(other))
			return wideningSameOperation(other);
		else
			return TOP;
	}

	protected abstract boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException;

	protected abstract DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException;

	protected abstract DataframeOperation wideningSameOperation(
			DataframeOperation other)
			throws SemanticException;

	@Override
	public int hashCode() {
		return Objects.hash(index, where);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DataframeOperation other = (DataframeOperation) obj;
		return index == other.index && Objects.equals(where, other.where);
	}

	@Override
	public abstract String toString();

	@Override
	public StructuredRepresentation representation() {
		return new StringRepresentation(toString());
	}

	@Override
	public final int compareTo(
			DataframeOperation o) {
		int cmp;
		if ((cmp = where.compareTo(o.where)) != 0)
			return cmp;
		if ((cmp = Integer.compare(index, o.index)) != 0)
			return cmp;
		if ((cmp = getClass().getName().compareTo(o.getClass().getName())) != 0)
			return cmp;
		return compareToSameOperation(o);
	}

	protected abstract int compareToSameOperation(
			DataframeOperation o);

	@Override
	public <V> boolean accept(
			GraphVisitor<DataframeForest, DataframeOperation, DataframeEdge, V> visitor,
			V tool) {
		return visitor.visit(tool, null, this);
	}

	@Override
	public int setOffset(
			int offset) {
		return this.offset = offset;
	}

	@Override
	public int getOffset() {
		return offset;
	}
}
