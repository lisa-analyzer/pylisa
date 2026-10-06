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
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.strings.StrStrip;

/**
 * Native implementation of {@code str.rstrip(self, chars=None)} and
 * {@code bytes.rstrip(self, chars=None)}: {@code chars} must have the type of
 * the receiver or be {@code None} (for whitespace, ASCII only for
 * {@code bytes}), otherwise {@code TypeError} is raised.
 */
public class StrRStrip extends PyNative {

	protected StrRStrip(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "rstrip", params);
	}

	public static StrRStrip build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrRStrip(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		for (boolean bytes : textModes(analysis, state, args[0])) {
			// the characters to remove have the type of the receiver (or None)
			Satisfiability typed = hasType(analysis, state, args[1], (bytes ? BYTES : STR).or(NONE));
			result = result.lub(typeChecked(analysis, state, typed, compute(analysis, state,
					new BinaryExpression(bytes ? PyBytesType.INSTANCE : StringType.INSTANCE, args[0], args[1],
							StrStrip.RSTRIP, getLocation()))));
		}
		return result;
	}
}
