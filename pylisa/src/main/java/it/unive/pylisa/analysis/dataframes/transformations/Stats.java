package it.unive.pylisa.analysis.dataframes.transformations;

public class Stats extends DataframeTransformation {

	public static final Stats INSTANCE = new Stats();
	
	private Stats() {
	}

	@Override
	public int hashCode() {
		return getClass().hashCode();
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "stats";
	}
}