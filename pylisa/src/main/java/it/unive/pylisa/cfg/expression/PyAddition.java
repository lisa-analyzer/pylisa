package it.unive.pylisa.cfg.expression;

import java.util.Collections;
import java.util.Set;

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
import it.unive.lisa.program.cfg.statement.numeric.Addition;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;

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
			InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, SymbolicExpression left,
			SymbolicExpression right, StatementStore<A> expressions) throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, this);		
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				if (tr.canBeAssignedTo(tl)) {
					// int + int (and subtypes thereof): call int.__add__,
					// falling back to int.__radd__ if it does not resolve
					UnresolvedCall add = new UnresolvedCall(
							getCFG(),
							getLocation(),
							CallType.STATIC,
							null,
							"__add__",
							LeftToRightEvaluation.INSTANCE,
							getLeft(),
							getRight());
					boolean addResolves;
					try {
						interprocedural.resolve(add,
								new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
						addResolves = true;
					} catch (CallResolutionException e) {
						addResolves = false;
					}

					if (addResolves)
						result = result.lub(add.forwardSemantics(state, interprocedural, expressions));
					else {
						UnresolvedCall radd = new UnresolvedCall(
								getCFG(),
								getLocation(),
								CallType.STATIC,
								null,
								"__radd__",
								LeftToRightEvaluation.INSTANCE,
								getRight(),
								getLeft());
						result = result.lub(radd.forwardSemantics(state, interprocedural, expressions));
					}
				}
			}
		}

		return result;
	}		
}
