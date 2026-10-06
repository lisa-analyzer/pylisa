package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.numeric.Addition;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code a + b}: it calls {@code type(a).__add__(a, b)}, falling back
 * to {@code type(b).__radd__(b, a)}; if neither applies, a {@code TypeError} is
 * raised. See {@link PyBinaryDispatch} for the details.
 */
public class PyAddition extends Addition {

	/**
	 * Builds the addition.
	 *
	 * @param cfg      the {@link CFG} where this operation lies
	 * @param location the location where this literal is defined
	 * @param left     the left-hand side of this operation
	 * @param right    the right-hand side of this operation
	 */
	public PyAddition(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, left, right);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		return PyBinaryDispatch.dispatch(interprocedural, state, expressions, this, left, right,
				"__add__", "__radd__", false, PyBinaryDispatch.Fallback.TYPE_ERROR);
	}

	@SuppressWarnings("unchecked")
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean resolves(
			InterproceduralAnalysis<A, D> interprocedural,
			UnresolvedCall call,
			Type self,
			Type other,
			SymbolAliasing aliasing) {
		try {
			// the call graph does not throw when no target is found, it
			// yields an OpenCall instead
			return !(interprocedural.resolve(call,
					new Set[] { Collections.singleton(self), Collections.singleton(other) },
					aliasing) instanceof OpenCall);
		} catch (CallResolutionException e) {
			return false;
		}
	}
}
