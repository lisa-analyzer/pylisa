package it.unive.pylisa.cfg.expression.comparison;

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
import it.unive.lisa.program.cfg.statement.comparison.NotEqual;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code !=}. Like {@code ==}, {@code __ne__} is its own reflection:
 * {@code a != b} calls {@code type(a).__ne__(a, b)}; if that returns
 * {@code NotImplemented}, it tries {@code type(b).__ne__(b, a)}; if neither
 * type implements the comparison, the result is {@code True} rather than an
 * exception (the mirror image of {@code ==} defaulting to {@code False}, since
 * both derive from the default identity-based comparison inherited from
 * {@code object}).
 */
public class PyNotEqual extends NotEqual {

	public PyNotEqual(
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
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS)) {
			AnalysisState<A> sem = PandasSemantics.compare(
					analysis,
					state,
					left,
					right,
					this,
					ComparisonOperator.NEQ);
			if (sem != null)
				return sem;
		}

		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				// type(a).__ne__(a, b)
				UnresolvedCall ne = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__ne__",
						LeftToRightEvaluation.INSTANCE,
						getLeft(),
						getRight());
				boolean neResolves;
				try {
					interprocedural.resolve(ne,
							new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
					neResolves = true;
				} catch (CallResolutionException e) {
					neResolves = false;
				}

				if (neResolves) {
					result = result.lub(ne.forwardSemantics(state, interprocedural, expressions));
					continue;
				}

				// type(a) does not implement it: try type(b).__ne__(b, a)
				UnresolvedCall rne = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__ne__",
						LeftToRightEvaluation.INSTANCE,
						getRight(),
						getLeft());
				boolean rneResolves;
				try {
					interprocedural.resolve(rne,
							new Set[] { Collections.singleton(tr), Collections.singleton(tl) }, aliasing);
					rneResolves = true;
				} catch (CallResolutionException e) {
					rneResolves = false;
				}

				if (rneResolves)
					result = result.lub(rne.forwardSemantics(state, interprocedural, expressions));
				else
					// neither type implements the comparison: True, not an
					// exception
					result = result.lub(analysis.smallStepSemantics(state,
							new Constant(getStaticType(), true, getLocation()), this));
			}
		}

		return result;
	}
}
