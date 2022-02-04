package it.unive.pylisa.analysis.dataframes.transformations;

import java.util.Objects;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

import org.apache.commons.lang3.StringUtils;

public class ProjectColumns extends DataframeTransformation {

	private final SortedSet<String> columns;

	public ProjectColumns(Set<String> columns) {
		if (columns instanceof SortedSet)
			this.columns = (SortedSet<String>) columns;
		else
			this.columns = new TreeSet<>(columns);
	}

	@Override
	public boolean equals(Object o) {
		if (this == o)
			return true;
		if (o == null || getClass() != o.getClass())
			return false;
		ProjectColumns that = (ProjectColumns) o;
		return Objects.equals(columns, that.columns);
	}

	@Override
	public int hashCode() {
		return Objects.hash(columns);
	}

	@Override
	public String toString() {
		return "project_cols:[" + StringUtils.join(columns, ", ") + "]";
	}
}