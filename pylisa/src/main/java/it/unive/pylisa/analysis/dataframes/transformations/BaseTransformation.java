package it.unive.pylisa.analysis.dataframes.transformations;

import java.util.Arrays;

public class BaseTransformation extends DataframeTransformation {

	private final String name;
	private final Object[] parameters;

	public BaseTransformation(String name, Object... parameters) {
		this.name = name;
		this.parameters = parameters;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((name == null) ? 0 : name.hashCode());
		result = prime * result + Arrays.deepHashCode(parameters);
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
		BaseTransformation other = (BaseTransformation) obj;
		if (name == null) {
			if (other.name != null)
				return false;
		} else if (!name.equals(other.name))
			return false;
		if (!Arrays.deepEquals(parameters, other.parameters))
			return false;
		return true;
	}



	@Override
	public String toString() {
		if (parameters.length == 0)
			return name;
		return name + ":" + Arrays.toString(parameters);
	}
}