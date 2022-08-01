package it.unive.pylisa.analysis.dataframes.graph;

public class NodeId {

	public static long counter = 0;

	private final long id;

	public NodeId() {
		this.id = counter++;
	}

	@Override
	public String toString() {
		return "node" + id;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (int) (id ^ (id >>> 32));
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
		NodeId other = (NodeId) obj;
		if (id != other.id)
			return false;
		return true;
	}
}
