package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.Codec;

/**
 * Native implementation of
 * {@code bytes.decode(self, encoding=None, errors=None)}. See
 * {@link CodecSemantics}.
 */
public class BytesDecode extends PyNative {

	protected BytesDecode(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "decode", params);
	}

	public static BytesDecode build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesDecode(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		return CodecSemantics.semantics(this, analysis, state, Codec.DECODE, args[0], args[1], args[2]);
	}
}
