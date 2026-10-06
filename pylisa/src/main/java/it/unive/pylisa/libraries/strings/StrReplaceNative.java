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
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.ternary.StringReplace;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.strings.ArgPair;
import it.unive.pylisa.symbolic.operators.strings.StrReplaceCount;

/**
 * Native implementation of {@code str.replace(self, old, new, count=None)}:
 * {@code old} and {@code new} must be {@code str}s and {@code count} an
 * {@code int}, otherwise {@code TypeError} is raised. A negative or omitted
 * {@code count} replaces all the occurrences.
 */
public class StrReplaceNative extends PyNative {

	protected StrReplaceNative(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "replace", params);
	}

	public static StrReplaceNative build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrReplaceNative(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability typed = hasType(analysis, state, args[1], STR)
				.and(hasType(analysis, state, args[2], STR))
				.and(hasType(analysis, state, args[3], INT.or(NONE)));
		SymbolicExpression value = args[3] instanceof PyNoneConstant
				? new TernaryExpression(StringType.INSTANCE, args[0], args[1], args[2], StringReplace.INSTANCE, loc)
				: new TernaryExpression(StringType.INSTANCE, args[0],
						new BinaryExpression(Untyped.INSTANCE, args[1], args[2], ArgPair.INSTANCE, loc),
						args[3], StrReplaceCount.INSTANCE, loc);
		return typeChecked(analysis, state, typed, compute(analysis, state, value));
	}
}
