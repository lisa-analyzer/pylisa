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
import it.unive.lisa.program.cfg.statement.comparison.Equal;
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
 * Python's {@code ==}. In real Python, {@code a == b} is value equality: it
 * calls {@code type(a).__eq__(a, b)}; if that returns {@code NotImplemented},
 * it tries the same {@code __eq__} method on {@code type(b)} with the operands
 * swapped ({@code type(b).__eq__(b, a)}) rather than a separate reflected
 * method name; if neither type implements the comparison, the result is
 * {@code False} rather than an exception.
 */
public class PyEquals extends Equal {

	public PyEquals(
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
					ComparisonOperator.EQ);
			if (sem != null)
				return sem;
		}

		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				// type(a).__eq__(a, b)
				UnresolvedCall eq = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__eq__",
						LeftToRightEvaluation.INSTANCE,
						getLeft(),
						getRight());
				boolean eqResolves;
				try {
					interprocedural.resolve(eq,
							new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
					eqResolves = true;
				} catch (CallResolutionException e) {
					eqResolves = false;
				}

				if (eqResolves) {
					result = result.lub(eq.forwardSemantics(state, interprocedural, expressions));
					continue;
				}

				// type(a) does not implement it: try type(b).__eq__(b, a)
				UnresolvedCall req = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__eq__",
						LeftToRightEvaluation.INSTANCE,
						getRight(),
						getLeft());
				boolean reqResolves;
				try {
					interprocedural.resolve(req,
							new Set[] { Collections.singleton(tr), Collections.singleton(tl) }, aliasing);
					reqResolves = true;
				} catch (CallResolutionException e) {
					reqResolves = false;
				}

				if (reqResolves)
					result = result.lub(req.forwardSemantics(state, interprocedural, expressions));
				else
					// neither type implements the comparison: False, not an
					// exception
					// TODO: use Python constant for False
					result = result.lub(analysis.smallStepSemantics(state,
							new Constant(getStaticType(), false, getLocation()), this));
			}
		}

		return result;
	}
}
