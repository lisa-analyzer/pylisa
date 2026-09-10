package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.UnsupportedStatementException;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code in} ({@code needle in container}). Real Python tries, in
 * order: (1) {@code type(container).__contains__(container, needle)}; (2)
 * otherwise, iterate {@code container} and compare each yielded element
 * against {@code needle} with {@code ==}; (3) otherwise, the old sequence
 * protocol, repeatedly indexing {@code container[0]}, {@code container[1]},
 * ... until {@code IndexError}. Only (1) is implemented here: mechanisms (2)
 * and (3) require iterating an abstractly-tracked container, which this
 * codebase does not support, so a runtime type pair without
 * {@code __contains__} throws {@link UnsupportedStatementException} rather
 * than attempting the iteration-based fallbacks.
 */
public class PyIn extends BinaryExpression {

	public PyIn(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "in", BoolType.INSTANCE, left, right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		// getLeft() is the needle, getRight() is the container: container.__contains__(needle)
		Set<Type> rtsContainer = analysis.getRuntimeTypesOf(state, right, this);
		Set<Type> rtsNeedle = analysis.getRuntimeTypesOf(state, left, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tContainer : rtsContainer) {
			for (Type tNeedle : rtsNeedle) {
				UnresolvedCall contains = null;
				for (CallType kind : new CallType[] { CallType.STATIC, CallType.INSTANCE }) {
					UnresolvedCall candidate = new UnresolvedCall(
							getCFG(),
							getLocation(),
							kind,
							null,
							"__contains__",
							LeftToRightEvaluation.INSTANCE,
							getRight(),
							getLeft());
					try {
						interprocedural.resolve(candidate,
								new Set[] { Collections.singleton(tContainer), Collections.singleton(tNeedle) },
								aliasing);
						contains = candidate;
						break;
					} catch (CallResolutionException e) {
						// try the next call kind
					}
				}

				if (contains == null)
					// no __contains__ for this type: real Python would fall back to
					// iterating the container, which is not modeled here
					throw new UnsupportedStatementException(this);
				result = result.lub(contains.forwardSemantics(state, interprocedural, expressions));
			}
		}

		return result;
	}
}
