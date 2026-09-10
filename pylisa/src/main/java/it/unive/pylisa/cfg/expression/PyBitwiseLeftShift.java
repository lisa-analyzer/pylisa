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
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import java.util.Collections;
import java.util.Set;

/**
 * Python's left shift ({@code a << b}). It evaluates {@code a} and {@code b},
 * then dispatches to {@code type(a).__lshift__(a, b)}, falling back to
 * {@code type(b).__rlshift__(b, a)} if needed. If neither supports it, that
 * type pair simply does not contribute to the result (there is no explicit
 * modeling of the {@code TypeError} raised in that case). A negative shift
 * count is handled by the underlying value domain (e.g. it yields bottom for
 * constant propagation), modeling the {@code ValueError} that Python raises for
 * a negative shift count.
 */
public class PyBitwiseLeftShift extends BinaryExpression {

	public PyBitwiseLeftShift(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "<<", Untyped.INSTANCE, left, right);
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
		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				if (tr.canBeAssignedTo(tl)) {
					// int << int (and subtypes thereof): call int.__lshift__,
					// falling back to int.__rlshift__ if it does not resolve
					UnresolvedCall lshift = new UnresolvedCall(
							getCFG(),
							getLocation(),
							CallType.STATIC,
							null,
							"__lshift__",
							LeftToRightEvaluation.INSTANCE,
							getLeft(),
							getRight());
					boolean lshiftResolves;
					try {
						interprocedural.resolve(lshift,
								new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
						lshiftResolves = true;
					} catch (CallResolutionException e) {
						lshiftResolves = false;
					}

					if (lshiftResolves)
						result = result.lub(lshift.forwardSemantics(state, interprocedural, expressions));
					else {
						UnresolvedCall rlshift = new UnresolvedCall(
								getCFG(),
								getLocation(),
								CallType.STATIC,
								null,
								"__rlshift__",
								LeftToRightEvaluation.INSTANCE,
								getRight(),
								getLeft());
						result = result.lub(rlshift.forwardSemantics(state, interprocedural, expressions));
					}
				}
			}
		}

		return result;
	}
}
