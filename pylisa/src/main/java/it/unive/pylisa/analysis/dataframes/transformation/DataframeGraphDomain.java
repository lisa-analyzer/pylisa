package it.unive.pylisa.analysis.dataframes.transformation;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.libraries.pandas.types.PandasType;
import java.util.Arrays;

public class DataframeGraphDomain extends BaseNonRelationalValueDomain<DataframeGraphDomain> {

	private static final DataframeOperation[] NO_TRANSFORMATIONS = new DataframeOperation[0];

	private static final DataframeGraphDomain BOTTOM = new DataframeGraphDomain(false);
	private static final DataframeGraphDomain TOP = new DataframeGraphDomain(true);

	private final DataframeOperation[] transformations;
	private final boolean isTop;

	public DataframeGraphDomain() {
		this(NO_TRANSFORMATIONS, true);
	}

	private DataframeGraphDomain(boolean isTop) {
		this(NO_TRANSFORMATIONS, isTop);
	}

	DataframeGraphDomain(DataframeOperation transformation) {
		this(new DataframeOperation[] { transformation }, false);
	}

	DataframeGraphDomain(DataframeGraphDomain source, DataframeOperation transformation) {
		this(append(source.transformations, transformation), false);
	}

	private static DataframeOperation[] append(DataframeOperation[] source,
			DataframeOperation transformation) {
		DataframeOperation[] copy = new DataframeOperation[source.length + 1];
		System.arraycopy(source, 0, copy, 0, source.length);
		copy[source.length] = transformation;
		return copy;
	}

	private DataframeGraphDomain(DataframeOperation[] transformations, boolean isTop) {
		this.transformations = transformations;
		this.isTop = isTop;
	}

	@Override
	protected DataframeGraphDomain lubAux(DataframeGraphDomain other) throws SemanticException {
		return equals(other) ? this : top();
	}

	@Override
	protected DataframeGraphDomain wideningAux(DataframeGraphDomain other) throws SemanticException {
		return lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeGraphDomain other) throws SemanticException {
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
		DataframeGraphDomain other = (DataframeGraphDomain) obj;
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
	public DataframeGraphDomain top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return transformations.length == 0 && isTop;
	}

	@Override
	public DataframeGraphDomain bottom() {
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
