package it.unive.pylisa.analysis.dataframes.transformation;

import java.util.Map.Entry;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.PairRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.analysis.NonRelationalValueCartesianProduct;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.transformations.BaseTransformation;
import it.unive.pylisa.symbolic.operators.ColumnAccess;
import it.unive.pylisa.symbolic.operators.ProjectRows;
import it.unive.pylisa.symbolic.operators.ReadDataframe;
import it.unive.pylisa.symbolic.operators.SetOptionAux;
import it.unive.pylisa.symbolic.operators.Statistics;
import it.unive.pylisa.symbolic.operators.StructuralInfo;
import it.unive.pylisa.symbolic.operators.TypeConversion;

public class DataframeDomain extends
		NonRelationalValueCartesianProduct<DataframeDomain, DataframeTransformationDomain, ConstantPropagation> {

	public DataframeDomain() {
		this(new DataframeTransformationDomain().top(), new ConstantPropagation().top());
	}

	public DataframeDomain(DataframeTransformationDomain left, ConstantPropagation right) {
		super(left, right);
	}

	@Override
	protected DataframeDomain mk(DataframeTransformationDomain left, ConstantPropagation right) {
		return new DataframeDomain(left, right);
	}

	@Override
	public DataframeDomain eval(ValueExpression expression, ValueEnvironment<DataframeDomain> environment,
			ProgramPoint pp) throws SemanticException {
		ValueEnvironment<DataframeTransformationDomain> lenv = new ValueEnvironment<>(left);
		ValueEnvironment<ConstantPropagation> renv = new ValueEnvironment<>(right);
		for (Entry<Identifier, DataframeDomain> entry : environment) {
			lenv = lenv.putState(entry.getKey(), entry.getValue().left);
			renv = renv.putState(entry.getKey(), entry.getValue().right);
		}

		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			DataframeDomain reduced = reducedUnary(pp, lenv, renv, unary);
			if (!reduced.isBottom())
				return reduced;
		} else if (expression instanceof BinaryExpression) {
			BinaryExpression ternary = (BinaryExpression) expression;
			DataframeDomain reduced = reducedBinary(pp, lenv, renv, ternary);
			if (!reduced.isBottom())
				return reduced;
		} else if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			DataframeDomain reduced = reducedTernary(pp, lenv, renv, ternary);
			if (!reduced.isBottom())
				return reduced;
		}

		return super.eval(expression, environment, pp);
	}

	private DataframeDomain reducedUnary(ProgramPoint pp, ValueEnvironment<DataframeTransformationDomain> lenv,
			ValueEnvironment<ConstantPropagation> renv, UnaryExpression unary) throws SemanticException {
		if (unary.getOperator() == ReadDataframe.INSTANCE) {
			ConstantPropagation filename = right.eval((ValueExpression) unary.getExpression(), renv, pp);
			if (topOrBottom(filename))
				return new DataframeDomain(left.top(), right.bottom());

			DataframeTransformationDomain df = new DataframeTransformationDomain(
					new BaseTransformation("file", filename.getConstantAs(String.class)));
			return new DataframeDomain(df, right.bottom());
		} else if (unary.getOperator() == Statistics.INSTANCE) {
			DataframeTransformationDomain df = extractDataFrame(unary.getExpression(), lenv, pp);
			DataframeTransformationDomain stat = new DataframeTransformationDomain(df,
					new BaseTransformation("stats"));
			return new DataframeDomain(stat, right.bottom());
		} else if (unary.getOperator() == StructuralInfo.INSTANCE) {
			DataframeTransformationDomain df = extractDataFrame(unary.getExpression(), lenv, pp);
			DataframeTransformationDomain info = new DataframeTransformationDomain(df,
					new BaseTransformation("info"));
			return new DataframeDomain(info, right.bottom());
		} else if (unary.getOperator() instanceof TypeConversion) {
			DataframeTransformationDomain df = extractDataFrame(unary.getExpression(), lenv, pp);
			DataframeTransformationDomain conv = new DataframeTransformationDomain(df,
					new BaseTransformation("conv", ((TypeConversion) unary.getOperator()).getType()));
			return new DataframeDomain(conv, right.bottom());
		} else
			return bottom();
	}

	private DataframeDomain reducedBinary(ProgramPoint pp, ValueEnvironment<DataframeTransformationDomain> lenv,
			ValueEnvironment<ConstantPropagation> renv, BinaryExpression binary) throws SemanticException {
		if (binary.getOperator() == ColumnAccess.INSTANCE) {
			DataframeTransformationDomain df = extractDataFrame(binary.getLeft(), lenv, pp);
			ConstantPropagation col = right.eval((ValueExpression) binary.getRight(), renv, pp);
			if (topOrBottom(df) || topOrBottom(col))
				return new DataframeDomain(left.top(), right.bottom());

			DataframeTransformationDomain ca = new DataframeTransformationDomain(df,
					new BaseTransformation("col_access", col.getConstantAs(String.class)));

			return new DataframeDomain(ca, right.bottom());
		} else
			return bottom();
	}

	private DataframeDomain reducedTernary(ProgramPoint pp, ValueEnvironment<DataframeTransformationDomain> lenv,
			ValueEnvironment<ConstantPropagation> renv, TernaryExpression ternary) throws SemanticException {
		if (ternary.getOperator() == ProjectRows.INSTANCE) {
			DataframeTransformationDomain df = extractDataFrame(ternary.getLeft(), lenv, pp);
			ConstantPropagation start = right.eval((ValueExpression) ternary.getMiddle(), renv, pp);
			ConstantPropagation end = right.eval((ValueExpression) ternary.getRight(), renv, pp);

			if (topOrBottom(df) || topOrBottom(start) || topOrBottom(end))
				return new DataframeDomain(left.top(), right.bottom());

			DataframeTransformationDomain pr = new DataframeTransformationDomain(df,
					new BaseTransformation("project_rows", start.getConstantAs(Integer.class),
							end.getConstantAs(Integer.class)));

			return new DataframeDomain(pr, right.bottom());
		} else if (ternary.getOperator() == SetOptionAux.INSTANCE) {
			DataframeTransformationDomain df = extractDataFrame(ternary.getLeft(), lenv, pp);
			ConstantPropagation key = right.eval((ValueExpression) ternary.getMiddle(), renv, pp);
			ConstantPropagation value = right.eval((ValueExpression) ternary.getRight(), renv, pp);

			if (topOrBottom(df) || topOrBottom(key) || topOrBottom(value))
				return new DataframeDomain(left.top(), right.bottom());

			DataframeTransformationDomain pr = new DataframeTransformationDomain(df,
					new BaseTransformation("set_opt", key.getConstantAs(String.class),
							value.getConstant()));

			return new DataframeDomain(pr, right.bottom());
		} else
			return bottom();
	}

	private DataframeTransformationDomain extractDataFrame(SymbolicExpression expr,
			ValueEnvironment<DataframeTransformationDomain> env, ProgramPoint pp) throws SemanticException {
		ValueExpression v = (ValueExpression) expr;
		if (v instanceof MemoryPointer)
			v = ((MemoryPointer) v).getReferencedLocation();
		return left.eval(v, env, pp);
	}

	private boolean topOrBottom(Lattice<?> l) {
		return l.isTop() || l.isBottom();
	}

	@Override
	public DomainRepresentation representation() {
		if (left.isTop() && right.isTop())
			return Lattice.TOP_REPR;

		if (left.isBottom() && right.isBottom())
			return Lattice.BOTTOM_REPR;

		if (topOrBottom(left) && !topOrBottom(right))
			return right.representation();
		if (!topOrBottom(left) && topOrBottom(right))
			return left.representation();
		return new PairRepresentation(left.representation(), right.representation());
	}
}
