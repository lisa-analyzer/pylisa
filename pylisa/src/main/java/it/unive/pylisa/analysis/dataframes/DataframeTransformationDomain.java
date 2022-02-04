package it.unive.pylisa.analysis.dataframes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.pylisa.analysis.dataframes.transformations.DataframeTransformation;
import it.unive.pylisa.analysis.dataframes.transformations.ReadFile;
import it.unive.pylisa.analysis.string.StringPropagation;
import it.unive.pylisa.symbolic.ReadDataframe;

public class DataframeTransformationDomain extends BaseNonRelationalValueDomain<DataframeTransformationDomain> {

	private static final DataframeTransformationDomain BOTTOM = new DataframeTransformationDomain(
			new ValueEnvironment<>(new StringPropagation()).bottom());
	private static final DataframeTransformationDomain TOP = new DataframeTransformationDomain();

	private final List<DataframeTransformation> transformations;

	private final ValueEnvironment<StringPropagation> strings;

	public DataframeTransformationDomain() {
		this(Collections.emptyList(), new ValueEnvironment<>(new StringPropagation()).top());
	}

	public DataframeTransformationDomain(ValueEnvironment<StringPropagation> strings) {
		this(Collections.emptyList(), strings);
	}

	private DataframeTransformationDomain(DataframeTransformation transformation,
			ValueEnvironment<StringPropagation> strings) {
		this(Collections.singletonList(transformation), strings);
	}

	private DataframeTransformationDomain(List<DataframeTransformation> transformations,
			ValueEnvironment<StringPropagation> strings) {
		this.transformations = transformations;
		this.strings = strings;
	}

	@Override
	public DataframeTransformationDomain eval(ValueExpression expression,
			ValueEnvironment<DataframeTransformationDomain> environment, ProgramPoint pp) throws SemanticException {
		DataframeTransformationDomain eval = super.eval(expression, environment, pp);
		return new DataframeTransformationDomain(eval.transformations, eval.strings.smallStepSemantics(expression, pp));
	}

	@Override
	protected DataframeTransformationDomain evalUnaryExpression(
			UnaryOperator operator,
			DataframeTransformationDomain arg,
			ProgramPoint pp) throws SemanticException {
		if (operator == ReadDataframe.INSTANCE) {
			StringPropagation filename = arg.strings.getValueOnStack();
			if (!filename.isBottom() && !filename.isTop())
				return new DataframeTransformationDomain(new ReadFile(filename.getConstant()), arg.strings);
		}
		return super.evalUnaryExpression(operator, arg, pp);
	}

	@Override
	protected DataframeTransformationDomain lubAux(DataframeTransformationDomain other) throws SemanticException {
		if (transformations.size() != other.transformations.size())
			return top();

		List<DataframeTransformation> newTransformation = lubTransformations(other);
		return new DataframeTransformationDomain(newTransformation, strings.lub(other.strings));
	}

	private List<DataframeTransformation> lubTransformations(DataframeTransformationDomain other)
			throws SemanticException {
		List<DataframeTransformation> newTransformation = new ArrayList<>(transformations.size());
		for (int i = 0; i < transformations.size(); i++)
			newTransformation.add(transformations.get(i).lub(other.transformations.get(i)));
		return newTransformation;
	}

	@Override
	protected DataframeTransformationDomain wideningAux(DataframeTransformationDomain other) throws SemanticException {
		if (transformations.size() != other.transformations.size())
			return top();

		List<DataframeTransformation> newTransformation = lubTransformations(other);
		return new DataframeTransformationDomain(newTransformation, strings.widening(other.strings));
	}

	@Override
	protected boolean lessOrEqualAux(DataframeTransformationDomain other) throws SemanticException {
		if (transformations.size() != other.transformations.size())
			return false;

		for (int i = 0; i < transformations.size(); i++)
			if (!transformations.get(i).lessOrEqual(other.transformations.get(i)))
				return false;

		return strings.lessOrEqual(other.strings);
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
		return transformations.isEmpty() && strings.isTop();
	}

	@Override
	public DataframeTransformationDomain bottom() {
		return DataframeTransformationDomain.BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return transformations.isEmpty() && strings.isBottom();
	}
}
