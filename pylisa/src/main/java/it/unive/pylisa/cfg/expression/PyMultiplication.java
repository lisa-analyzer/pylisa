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
import it.unive.lisa.program.cfg.statement.numeric.Multiplication;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyLibraryUnitType;
import it.unive.pylisa.symbolic.operators.StringMult;
import java.util.Collections;
import java.util.Set;

public class PyMultiplication extends Multiplication {

	public PyMultiplication(
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

		if (rtsl.stream().anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, true))
				|| rtsr.stream().anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, true)))
			// we allow scalar multiplication, but with no explicit handling for
			// now
			return state;

		// string repeat: STRING * Integer || Integer * String
		if ((rtsl.stream().anyMatch(Type::isStringType) && rtsr.stream().anyMatch(Type::isNumericType)) ||
				(rtsr.stream().anyMatch(Type::isStringType) && rtsl.stream().anyMatch(Type::isNumericType))) {
			return analysis.smallStepSemantics(state,
					new BinaryExpression(
							getStaticType(),
							left,
							right,
							StringMult.INSTANCE,
							getLocation()),
					this);
		}

		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				if (tr.canBeAssignedTo(tl)) {
					// int * int (and subtypes thereof): call int.__mul__,
					// falling back to int.__rmul__ if it does not resolve
					UnresolvedCall mul = new UnresolvedCall(
							getCFG(),
							getLocation(),
							CallType.STATIC,
							null,
							"__mul__",
							LeftToRightEvaluation.INSTANCE,
							getLeft(),
							getRight());
					boolean mulResolves;
					try {
						interprocedural.resolve(mul,
								new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
						mulResolves = true;
					} catch (CallResolutionException e) {
						mulResolves = false;
					}

					if (mulResolves)
						result = result.lub(mul.forwardSemantics(state, interprocedural, expressions));
					else {
						UnresolvedCall rmul = new UnresolvedCall(
								getCFG(),
								getLocation(),
								CallType.STATIC,
								null,
								"__rmul__",
								LeftToRightEvaluation.INSTANCE,
								getRight(),
								getLeft());
						result = result.lub(rmul.forwardSemantics(state, interprocedural, expressions));
					}
				}
			}
		}

		return result;
	}
}
