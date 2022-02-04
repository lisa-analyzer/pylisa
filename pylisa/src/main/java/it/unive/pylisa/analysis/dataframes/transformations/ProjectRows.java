package it.unive.pylisa.analysis.dataframes.transformations;

public class ProjectRows extends DataframeTransformation {

	private final int start;
	private final int end;

	public ProjectRows(int start, int end) {
		this.start = start;
		this.end = end;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + end;
		result = prime * result + start;
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
		ProjectRows other = (ProjectRows) obj;
		if (end != other.end)
			return false;
		if (start != other.start)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "project_rows:[" + start + ":" + end + "]";
	}
}