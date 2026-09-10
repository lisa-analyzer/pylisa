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
import it.unive.lisa.program.cfg.statement.comparison.GreaterThan;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.pandas.PandasSemantics;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGt;
import it.unive.pylisa.symbolic.operators.dataframes.aux.ComparisonOperator;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code >}. It calls {@code type(a).__gt__(a, b)}; if that returns
 * {@code NotImplemented}, it tries the reflected comparison on the other
 * operand's type, {@code type(b).__lt__(b, a)} ({@code __gt__} and
 * {@code __lt__} are each other's reflection, unlike {@code __eq__} which is
 * its own reflection). If neither type implements the comparison for a given
 * runtime type pair, real Python raises {@code TypeError}; this codebase does
 * not model exceptions, so that pair falls back to the previous, type-agnostic
 * direct comparison instead, preserving behavior for types that have not been
 * hooked into the dunder-dispatch system yet.
 */
public class PyGreaterThan extends GreaterThan {

	public PyGreaterThan(
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
					ComparisonOperator.GT);
			if (sem != null)
				return sem;
		}

		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				// type(a).__gt__(a, b)
				UnresolvedCall gt = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__gt__",
						LeftToRightEvaluation.INSTANCE,
						getLeft(),
						getRight());
				boolean gtResolves;
				try {
					interprocedural.resolve(gt,
							new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
					gtResolves = true;
				} catch (CallResolutionException e) {
					gtResolves = false;
				}

				if (gtResolves) {
					result = result.lub(gt.forwardSemantics(state, interprocedural, expressions));
					continue;
				}

				// type(a) does not implement it: try the reflection,
				// type(b).__lt__(b, a)
				UnresolvedCall lt = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__lt__",
						LeftToRightEvaluation.INSTANCE,
						getRight(),
						getLeft());
				boolean ltResolves;
				try {
					interprocedural.resolve(lt,
							new Set[] { Collections.singleton(tr), Collections.singleton(tl) }, aliasing);
					ltResolves = true;
				} catch (CallResolutionException e) {
					ltResolves = false;
				}

				if (ltResolves)
					result = result.lub(lt.forwardSemantics(state, interprocedural, expressions));
				else
					// neither type implements it yet: fall back to the
					// previous,
					// type-agnostic direct comparison
					result = result.lub(analysis.smallStepSemantics(state,
							new BinaryExpression(BoolType.INSTANCE, left, right, PyComparisonGt.INSTANCE,
									getLocation()),
							this));
			}
		}

		return result;
	}
}
