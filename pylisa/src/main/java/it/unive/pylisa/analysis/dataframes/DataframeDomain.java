package it.unive.pylisa.analysis.dataframes;

import java.util.Map.Entry;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.analysis.NonRelationalValueCartesianProduct;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformations.ReadFile;
import it.unive.pylisa.symbolic.ProjectRows;
import it.unive.pylisa.symbolic.ReadDataframe;

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
			if (unary.getOperator() == ReadDataframe.INSTANCE) {
				ConstantPropagation filename = right.eval((ValueExpression) unary.getExpression(), renv, pp);
				if (topOrBottom(filename))
					return new DataframeDomain(left.top(), right.bottom());

				DataframeTransformationDomain df = new DataframeTransformationDomain(
						new ReadFile(filename.getConstantAs(String.class)));
				return new DataframeDomain(df, right.bottom());
			}
		}

		if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			if (ternary.getOperator() == ProjectRows.INSTANCE) {
				ValueExpression v = (ValueExpression) ternary.getLeft();
				if (v instanceof MemoryPointer)
					v = ((MemoryPointer) v).getReferencedLocation();
				DataframeTransformationDomain df = left.eval(v, lenv, pp);
				ConstantPropagation start = right.eval((ValueExpression) ternary.getMiddle(), renv, pp);
				ConstantPropagation end = right.eval((ValueExpression) ternary.getRight(), renv, pp);

				if (topOrBottom(df) || topOrBottom(start) || topOrBottom(end))
					return new DataframeDomain(left.top(), right.bottom());

				DataframeTransformationDomain pr = new DataframeTransformationDomain(df,
						new it.unive.pylisa.analysis.dataframes.transformations.ProjectRows(
								start.getConstantAs(Integer.class), end.getConstantAs(Integer.class)));

				return new DataframeDomain(pr, right.bottom());
			}
		}

		return super.eval(expression, environment, pp);
	}

	private boolean topOrBottom(Lattice<?> l) {
		return l.isTop() || l.isBottom();
	}

	@Override
	public DomainRepresentation representation() {
		return left.representation();
	}
}
