package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.strings.StrStrip;

/**
 * Native implementation of {@code str.lstrip(self, chars=None)}: {@code chars}
 * must be a {@code str} or {@code None}, otherwise {@code TypeError} is raised.
 */
public class StrLStrip extends PyNative {

	protected StrLStrip(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "lstrip", params);
	}

	public static StrLStrip build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrLStrip(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Satisfiability typed = hasType(analysis, state, args[1], STR.or(NONE));
		return typeChecked(analysis, state, typed, compute(analysis, state,
				new BinaryExpression(StringType.INSTANCE, args[0], args[1], StrStrip.LSTRIP, getLocation())));
	}
}
