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
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.numeric.Division;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.Set;

/**
 * Python's true division ({@code a / b}). It evaluates {@code a} and
 * {@code b}, then dispatches to {@code type(a).__truediv__(a, b)}, falling
 * back to {@code type(b).__rtruediv__(b, a)} if needed. If neither supports
 * it, that type pair simply does not contribute to the result (there is no
 * explicit modeling of the {@code TypeError} raised in that case). Division
 * by zero is handled by the underlying value domain (e.g. it yields bottom
 * for constant propagation), modeling the {@code ZeroDivisionError} that
 * Python raises for {@code int}/{@code float} operands.
 */
public class PyDivision extends Division {

	/**
	 * Builds the division.
	 *
	 * @param cfg      the {@link CFG} where this operation lies
	 * @param location the location where this literal is defined
	 * @param left     the left-hand side of this operation
	 * @param right    the right-hand side of this operation
	 */
	public PyDivision(
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				if (tr.canBeAssignedTo(tl)) {
					// int / int (and subtypes thereof): call int.__truediv__,
					// falling back to int.__rtruediv__ if it does not resolve
					UnresolvedCall div = new UnresolvedCall(
							getCFG(),
							getLocation(),
							CallType.STATIC,
							null,
							"__truediv__",
							LeftToRightEvaluation.INSTANCE,
							getLeft(),
							getRight());
					boolean divResolves;
					try {
						interprocedural.resolve(div,
								new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
						divResolves = true;
					} catch (CallResolutionException e) {
						divResolves = false;
					}

					if (divResolves)
						result = result.lub(div.forwardSemantics(state, interprocedural, expressions));
					else {
						UnresolvedCall rdiv = new UnresolvedCall(
								getCFG(),
								getLocation(),
								CallType.STATIC,
								null,
								"__rtruediv__",
								LeftToRightEvaluation.INSTANCE,
								getRight(),
								getLeft());
						result = result.lub(rdiv.forwardSemantics(state, interprocedural, expressions));
					}
				}
			}
		}

		return result;
	}
}
