package it.unive.pylisa.analysis.dataframes;

import java.util.Map.Entry;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.analysis.NonRelationalValueCartesianProduct;
import it.unive.pylisa.analysis.string.StringPropagation;
import it.unive.pylisa.symbolic.ReadDataframe;

public class DataframeDomain extends
		NonRelationalValueCartesianProduct<DataframeTransformationDomain, StringPropagation> {

	public DataframeDomain() {
		this(new DataframeTransformationDomain().top(), new StringPropagation().top());
	}

	public DataframeDomain(DataframeTransformationDomain left, StringPropagation right) {
		super(left, right);
	}

	@Override
	protected DataframeDomain mk(DataframeTransformationDomain left, StringPropagation right) {
		return new DataframeDomain(left, right);
	}

	@Override
	public NonRelationalValueCartesianProduct<DataframeTransformationDomain, StringPropagation> eval(
			ValueExpression expression,
			ValueEnvironment<
					NonRelationalValueCartesianProduct<DataframeTransformationDomain, StringPropagation>> environment,
			ProgramPoint pp) throws SemanticException {
		ValueEnvironment<DataframeTransformationDomain> lenv = new ValueEnvironment<>(left);
		ValueEnvironment<StringPropagation> renv = new ValueEnvironment<>(right);
		for (Entry<Identifier, NonRelationalValueCartesianProduct<DataframeTransformationDomain,
				StringPropagation>> entry : environment) {
			lenv.putState(entry.getKey(), entry.getValue().left);
			renv.putState(entry.getKey(), entry.getValue().right);
		}

		if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			if (unary.getOperator() instanceof ReadDataframe) {
				StringPropagation filename = right.eval((ValueExpression) unary.getExpression(), renv, pp);
				DataframeTransformationDomain df = left.reducedEvalUnary(unary.getOperator(), left.bottom(),
						filename, pp);
				return new DataframeDomain(df, right.bottom());
			}
		}

		return super.eval(expression, environment, pp);
	}
}
