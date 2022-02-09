package it.unive.pylisa.analysis.dataframes;

public interface DataframeAwareDomain<T extends DataframeAwareDomain<T, D>, D> {
	D getDataFrame();

	boolean sameDataFrame(D other);

	T createDataframe(D value);
}
