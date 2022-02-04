package it.unive.pylisa.analysis.dataframes;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.analysis.dataframes.transformations.DataframeTransformation;
import it.unive.pylisa.libraries.pandas.PyDataframeType;

public class DataframeTransformationDomain extends BaseNonRelationalValueDomain<DataframeTransformationDomain> {

	private static final DataframeTransformationDomain BOTTOM = new DataframeTransformationDomain(false);
	private static final DataframeTransformationDomain TOP = new DataframeTransformationDomain(true);

	private final List<DataframeTransformation> transformations;
	private final boolean isTop;

	public DataframeTransformationDomain() {
		this(Collections.emptyList(), true);
	}

	public DataframeTransformationDomain(boolean isTop) {
		this(Collections.emptyList(), isTop);
	}

	DataframeTransformationDomain(DataframeTransformation transformation) {
		this(Collections.singletonList(transformation), false);
	}

	DataframeTransformationDomain(DataframeTransformationDomain source, DataframeTransformation transformation) {
		this(append(source.transformations, transformation), false);
	}

	private static List<DataframeTransformation> append(List<DataframeTransformation> source,
			DataframeTransformation transformation) {
		List<DataframeTransformation> copy = new LinkedList<>(source);
		copy.add(transformation);
		return copy;
	}

	private DataframeTransformationDomain(List<DataframeTransformation> transformations, boolean isTop) {
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
		result = prime * result + ((transformations == null) ? 0 : transformations.hashCode());
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
		if (transformations == null) {
			if (other.transformations != null)
				return false;
		} else if (!transformations.equals(other.transformations))
			return false;
		return true;
	}

	@Override
	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.TOP_REPR;
		if (isBottom())
			return Lattice.BOTTOM_REPR;
		return new StringRepresentation(Arrays.toString(transformations.toArray()));
	}

	@Override
	public DataframeTransformationDomain top() {
		return DataframeTransformationDomain.TOP;
	}

	@Override
	public boolean isTop() {
		return transformations.isEmpty() && isTop;
	}

	@Override
	public DataframeTransformationDomain bottom() {
		return DataframeTransformationDomain.BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return transformations.isEmpty() && !isTop;
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return canProcess(id);
	}

	@Override
	public boolean canProcess(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes().anyMatch(t -> t.equals(PyDataframeType.INSTANCE))
				: expression.getStaticType().equals(PyDataframeType.INSTANCE) || expression.getStaticType().isUntyped();
	}
}
