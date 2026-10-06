package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.StringToUpperCase;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code str.upper(self)}.
 */
public class StrUpper extends PyNative {

	protected StrUpper(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "upper", params);
	}

	public static StrUpper build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrUpper(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return compute(analysis, state,
				new UnaryExpression(StringType.INSTANCE, args[0], StringToUpperCase.INSTANCE, getLocation()));
	}
}
