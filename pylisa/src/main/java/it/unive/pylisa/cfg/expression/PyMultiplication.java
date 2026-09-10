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
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code *}. It calls {@code type(a).__mul__(a, b)}, falling back
 * to {@code type(b).__rmul__(b, a)} if needed. Unlike the other arithmetic
 * operators, {@code *} is genuinely asymmetric between types (e.g.
 * {@code "x" * 3} and {@code [1] * 3}: the left operand's type governs the
 * result, but the right operand need not be assignable to it), so no
 * {@code canBeAssignedTo} gate is applied before dispatch. Note that this
 * codebase's call resolution keeps matching {@code int.__mul__(3, "x")}
 * ahead of the reflected {@code str.__rmul__}, regardless of {@code other}'s
 * declared type, so {@code IntMul}/{@code IntRMul} handle the string-repeat
 * case internally rather than relying on the reflected fallback ever being
 * reached for that pair.
 */
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
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl) {
			for (Type tr : rtsr) {
				// type(a).__mul__(a, b)
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

				if (mulResolves) {
					AnalysisState<A> mulResult = mul.forwardSemantics(state, interprocedural, expressions);
					if (!mulResult.isBottom()) {
						result = result.lub(mulResult);
						continue;
					}
					// __mul__ "resolved" (its declared parameter types are
					// permissive on purpose, see IntMul) but could not actually
					// compute anything for this pair of types: fall through and
					// also try the reflected __rmul__, mirroring NotImplemented
				}

				// type(a) does not implement it: try type(b).__rmul__(b, a)
				UnresolvedCall rmul = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__rmul__",
						LeftToRightEvaluation.INSTANCE,
						getRight(),
						getLeft());
				try {
					interprocedural.resolve(rmul,
							new Set[] { Collections.singleton(tr), Collections.singleton(tl) }, aliasing);
					result = result.lub(rmul.forwardSemantics(state, interprocedural, expressions));
				} catch (CallResolutionException e) {
					// neither type implements *: this pair does not contribute to the result
				}
			}
		}

		return result;
	}
}
