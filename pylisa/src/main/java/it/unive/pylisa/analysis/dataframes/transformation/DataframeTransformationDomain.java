package it.unive.pylisa.analysis.dataframes.transformation;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.analysis.dataframes.transformation.transformations.DataframeTransformation;
import it.unive.pylisa.libraries.pandas.types.PandasType;
import java.util.Arrays;

public class DataframeTransformationDomain extends BaseNonRelationalValueDomain<DataframeTransformationDomain> {

	private static final DataframeTransformation[] NO_TRANSFORMATIONS = new DataframeTransformation[0];

	private static final DataframeTransformationDomain BOTTOM = new DataframeTransformationDomain(false);
	private static final DataframeTransformationDomain TOP = new DataframeTransformationDomain(true);

	private final DataframeTransformation[] transformations;
	private final boolean isTop;

	public DataframeTransformationDomain() {
		this(NO_TRANSFORMATIONS, true);
	}

	private DataframeTransformationDomain(boolean isTop) {
		this(NO_TRANSFORMATIONS, isTop);
	}

	DataframeTransformationDomain(DataframeTransformation transformation) {
		this(new DataframeTransformation[] { transformation }, false);
	}

	DataframeTransformationDomain(DataframeTransformationDomain source, DataframeTransformation transformation) {
		this(append(source.transformations, transformation), false);
	}

	private static DataframeTransformation[] append(DataframeTransformation[] source,
			DataframeTransformation transformation) {
		DataframeTransformation[] copy = new DataframeTransformation[source.length + 1];
		System.arraycopy(source, 0, copy, 0, source.length);
		copy[source.length] = transformation;
		return copy;
	}

	private DataframeTransformationDomain(DataframeTransformation[] transformations, boolean isTop) {
		this.transformations = transformations;
		this.isTop = isTop;
	}

	@Override
	protected DataframeTransformationDomain lubAux(DataframeTransformationDomain other) throws SemanticException {
		return equals(other) ? this : top();
	}

	@Override
	protected DataframeTransformationDomain wideningAux(DataframeTransformationDomain other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeTransformationDomain other) throws SemanticException {
		return equals(other);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + (isTop ? 1231 : 1237);
		result = prime * result + Arrays.hashCode(transformations);
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
		DataframeTransformationDomain other = (DataframeTransformationDomain) obj;
		if (isTop != other.isTop)
			return false;
		if (!Arrays.equals(transformations, other.transformations))
			return false;
		return true;
	}

	@Override
	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;
		if (isBottom())
			return Lattice.BOTTOM_REPR;
		return new StringRepresentation(Arrays.toString(transformations));
	}

	@Override
	public DataframeTransformationDomain top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return transformations.length == 0 && isTop;
	}

	@Override
	public DataframeTransformationDomain bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return transformations.length == 0 && !isTop;
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return canProcess(id);
	}

	@Override
	public boolean canProcess(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes().anyMatch(PandasType.class::isInstance)
				: expression.getStaticType() instanceof PandasType || expression.getStaticType().isUntyped();
	}
}
