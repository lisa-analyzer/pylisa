package it.unive.pylisa.analysis.dataframes;

import java.util.Objects;

import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public class NodeId {

	private final String op;

	private final int index;

	private final int locHash;

	public NodeId(
			DataframeOperation node) {
		this.op = node.getClass().getSimpleName();
		this.locHash = node.getWhere().getCodeLocation().hashCode();
		this.index = node.getIndex();
	}

	public NodeId(
			NodeId other,
			int index) {
		this.op = other.op;
		this.locHash = other.locHash;
		this.index = index;
	}

	@Override
	public String toString() {
		return op + "-" + locHash + "-" + index;
	}

	@Override
	public int hashCode() {
		return Objects.hash(index, locHash, op);
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
		NodeId other = (NodeId) obj;
		return index == other.index && locHash == other.locHash && Objects.equals(op, other.op);
	}
}
