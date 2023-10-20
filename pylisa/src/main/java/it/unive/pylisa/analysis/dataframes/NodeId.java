package it.unive.pylisa.analysis.dataframes;

import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;

public class NodeId {

	private final String id;

	public NodeId(
			DataframeOperation node) {
		this.id = "node" + node.getOffset() + "-" + node.getWhere().getCodeLocation().hashCode();
	}

	public NodeId(
			NodeId other) {
		// we converge by only allowing one level of copies
		this.id = other.id.startsWith("copy") ? other.id : "copy-" + other.id;
	}

	@Override
	public String toString() {
		return id;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((id == null) ? 0 : id.hashCode());
		return result;
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
		if (id == null) {
			if (other.id != null)
				return false;
		} else if (!id.equals(other.id))
			return false;
		return true;
	}
}
