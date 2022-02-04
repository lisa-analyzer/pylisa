package it.unive.pylisa.analysis;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.symbolic.DataFrameConstant;

public class DataframeTransformationDomain extends BaseNonRelationalValueDomain<DataframeTransformationDomain> {
	private final List<DataframeSingleTransformation> transformations;
	private static final DataframeTransformationDomain bottom = new DataframeTransformationDomain();
	private static final DataframeTransformationDomain top = new DataframeTransformationDomain(new ArrayList<>());

	private DataframeTransformationDomain(List<DataframeSingleTransformation> transformations) {
		this.transformations = transformations;
	}

	public DataframeTransformationDomain(DataframeSingleTransformation transformation) {
		this.transformations = new ArrayList<>();
		this.transformations.add(transformation);
	}

	private DataframeTransformationDomain() {
		this.transformations = null;
	}

	@Override
	protected DataframeTransformationDomain evalNonNullConstant(Constant constant, ProgramPoint pp)
			throws SemanticException {
		if (constant instanceof DataFrameConstant) {
			SymbolicExpression s = ((DataFrameConstant) constant).getValue();
			if (s instanceof Constant) {
				Object value = ((Constant) s).getValue();
				if (value instanceof String)
					return new DataframeTransformationDomain(
							new DataframeSingleTransformation.Dataframe((String) value));
			}
		}
		return super.evalNonNullConstant(constant, pp);
	}

	@Override
	protected DataframeTransformationDomain lubAux(DataframeTransformationDomain other) throws SemanticException {
		if (this.transformations.size() != other.transformations.size())
			return top();
		List<DataframeSingleTransformation> newTransformation = new ArrayList<>();
		for (int i = 0; i < this.transformations.size(); i++)
			newTransformation.add(this.transformations.get(i).lub(other.transformations.get(i)));
		return new DataframeTransformationDomain(newTransformation);
	}

	@Override
	protected DataframeTransformationDomain wideningAux(DataframeTransformationDomain other) throws SemanticException {
		return this.lubAux(other);
	}

	@Override
	protected boolean lessOrEqualAux(DataframeTransformationDomain other) throws SemanticException {
		if (this.transformations.size() != other.transformations.size())
			return false;
		for (int i = 0; i < this.transformations.size(); i++)
			if (!(this.transformations.get(i).lessOrEqual(other.transformations.get(i))))
				return false;
		return true;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o)
			return true;
		if (o == null || getClass() != o.getClass())
			return false;
		DataframeTransformationDomain that = (DataframeTransformationDomain) o;
		return Objects.equals(transformations, that.transformations);
	}

	@Override
	public int hashCode() {
		return Objects.hash(transformations);
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
		return DataframeTransformationDomain.top;
	}

	@Override
	public DataframeTransformationDomain bottom() {
		return DataframeTransformationDomain.bottom;
	}
}
