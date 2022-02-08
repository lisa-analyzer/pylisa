package it.unive.pylisa.analysis.dataframes.structure;

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
import it.unive.pylisa.analysis.dataframes.DataframeAwareDomain;
import it.unive.pylisa.symbolic.operators.ColumnAccess;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class DataframeStructureDomain extends
		NonRelationalValueCartesianProduct<DataframeStructureDomain, SingleDataframe, ConstantPropagation>
		implements DataframeAwareDomain<DataframeStructureDomain, SingleDataframe> {

	public DataframeStructureDomain() {
		this(new SingleDataframe(), new ConstantPropagation());
	}

	public DataframeStructureDomain(SingleDataframe left, ConstantPropagation right) {
		super(left, right);
	}

	@Override
	protected DataframeStructureDomain mk(SingleDataframe left, ConstantPropagation right) {
		return new DataframeStructureDomain(left, right);
	}

	@Override
	public DataframeStructureDomain eval(ValueExpression expression,
			ValueEnvironment<DataframeStructureDomain> environment,
			ProgramPoint pp) throws SemanticException {
		ValueEnvironment<SingleDataframe> lenv = new ValueEnvironment<>(left);
		ValueEnvironment<ConstantPropagation> renv = new ValueEnvironment<>(right);
		for (Entry<Identifier, DataframeStructureDomain> entry : environment) {
			lenv = lenv.putState(entry.getKey(), entry.getValue().left);
			renv = renv.putState(entry.getKey(), entry.getValue().right);
		}

		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			DataframeStructureDomain reduced = reducedUnary(pp, lenv, renv, unary);
			if (!reduced.isBottom())
				return reduced;
		} else if (expression instanceof BinaryExpression) {
			BinaryExpression ternary = (BinaryExpression) expression;
			DataframeStructureDomain reduced = reducedBinary(pp, lenv, renv, ternary);
			if (!reduced.isBottom())
				return reduced;
		} else if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			DataframeStructureDomain reduced = reducedTernary(pp, lenv, renv, ternary);
			if (!reduced.isBottom())
				return reduced;
		}

		return super.eval(expression, environment, pp);
	}

	private DataframeStructureDomain reducedUnary(ProgramPoint pp, ValueEnvironment<SingleDataframe> lenv,
			ValueEnvironment<ConstantPropagation> renv, UnaryExpression unary) throws SemanticException {
		if (unary.getOperator() == ReadDataframe.INSTANCE) {
			ConstantPropagation filename = right.eval((ValueExpression) unary.getExpression(), renv, pp);
			if (topOrBottom(filename))
				return new DataframeStructureDomain(left.top(), right.bottom());

			SingleDataframe df = new SingleDataframe(filename.getConstantAs(String.class));
			return new DataframeStructureDomain(df, right.bottom());
		} else
			return bottom();
	}

	private DataframeStructureDomain reducedBinary(ProgramPoint pp, ValueEnvironment<SingleDataframe> lenv,
			ValueEnvironment<ConstantPropagation> renv, BinaryExpression binary) throws SemanticException {
		if (binary.getOperator() == ColumnAccess.INSTANCE) {
			SingleDataframe df = extractDataFrame(binary.getLeft(), lenv, pp);
			ConstantPropagation col = right.eval((ValueExpression) binary.getRight(), renv, pp);
			if (topOrBottom(df) || topOrBottom(col))
				return new DataframeStructureDomain(left.top(), right.bottom());

			SingleDataframe ca = new SingleDataframe(df).addColumn(col.getConstantAs(String.class), true);
			return new DataframeStructureDomain(ca, right.bottom());
		} else
			return bottom();
	}

	private DataframeStructureDomain reducedTernary(ProgramPoint pp, ValueEnvironment<SingleDataframe> lenv,
			ValueEnvironment<ConstantPropagation> renv, TernaryExpression ternary) throws SemanticException {
//		if (ternary.getOperator() == ProjectRows.INSTANCE) {
//			SingleDataframe df = extractDataFrame(ternary.getLeft(), lenv, pp);
//			ConstantPropagation start = right.eval((ValueExpression) ternary.getMiddle(), renv, pp);
//			ConstantPropagation end = right.eval((ValueExpression) ternary.getRight(), renv, pp);
//
//			if (topOrBottom(df) || topOrBottom(start) || topOrBottom(end))
//				return new DataframeStructureDomain(left.top(), right.bottom());
//
//			SingleDataframe pr = new SingleDataframe(df,
//					new BaseTransformation("project_rows", start.getConstantAs(Integer.class),
//							end.getConstantAs(Integer.class)));
//
//			return new DataframeStructureDomain(pr, right.bottom());
//		} else
		return bottom();
	}

	private SingleDataframe extractDataFrame(SymbolicExpression expr,
			ValueEnvironment<SingleDataframe> env, ProgramPoint pp) throws SemanticException {
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

	@Override
	public SingleDataframe getDataFrame() {
		return left;
	}

	@Override
	public boolean sameDataFrame(SingleDataframe other) {
		return left.equals(other);
	}

	@Override
	public DataframeStructureDomain createDataframe(SingleDataframe value) {
		return new DataframeStructureDomain(value, right.bottom());
	}
}
