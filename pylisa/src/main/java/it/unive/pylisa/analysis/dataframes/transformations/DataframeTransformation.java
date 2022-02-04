package it.unive.pylisa.analysis.dataframes.transformations;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.SemanticException;

public abstract class DataframeTransformation extends BaseLattice<DataframeTransformation> {

	private static final DataframeTransformation TOP = new TopTransformation();

	private static final DataframeTransformation BOTTOM = new BottomTransformation();

	@Override
	public final DataframeTransformation top() {
		return TOP;
	}

	@Override
	public final DataframeTransformation bottom() {
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
	protected DataframeTransformation lubAux(DataframeTransformation other) throws SemanticException {
		return equals(other) ? this : top();
	}

	@Override
	protected final DataframeTransformation wideningAux(DataframeTransformation other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeTransformation other) throws SemanticException {
		return equals(other);
	}
}
