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
 * Python's {@code %}. On numbers it is the remainder operator,
 * {@code a - floor(a / b) * b} (sign of the divisor, unlike Java's
 * {@code %}); on a string left-hand side it is percent-formatting
 * ({@code "%s" % x}) &mdash; both are modeled through the same
 * {@code __mod__} dunder, since real Python dispatches {@code "%s" % x} to
 * {@code str.__mod__("%s", x)} exactly like any other {@code %} call. It
 * evaluates {@code a} and {@code b}, then dispatches to
 * {@code type(a).__mod__(a, b)}, falling back to
 * {@code type(b).__rmod__(b, a)} if needed (this codebase has no notion of
 * subclassing between library-defined types, so the "proper subclass with an
 * overriding reflected method" priority rule does not apply here). If
 * neither supports it, that type pair simply does not contribute to the
 * result (there is no explicit modeling of the {@code TypeError} raised in
 * that case).
 */
public class PyRemainder extends BinaryExpression {

	public PyRemainder(
			CFG cfg,
			CodeLocation location,
			Expression left,
			Expression right) {
		super(cfg, location, "%", Untyped.INSTANCE, left, right);
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
				// type(a).__mod__(a, b) -- e.g. int % int, or str.__mod__ for
				// string formatting ("%s" % x), where b need not be of a
				// type related to a, so no canBeAssignedTo gate is applied
				// here (unlike the other, symmetric arithmetic operators)
				UnresolvedCall mod = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__mod__",
						LeftToRightEvaluation.INSTANCE,
						getLeft(),
						getRight());
				boolean modResolves;
				try {
					interprocedural.resolve(mod,
							new Set[] { Collections.singleton(tl), Collections.singleton(tr) }, aliasing);
					modResolves = true;
				} catch (CallResolutionException e) {
					modResolves = false;
				}

				if (modResolves) {
					result = result.lub(mod.forwardSemantics(state, interprocedural, expressions));
					continue;
				}

				// type(a) does not implement it: try type(b).__rmod__(b, a)
				// (there is no __rmod__ for str, so this only ever resolves
				// for the numeric types)
				UnresolvedCall rmod = new UnresolvedCall(
						getCFG(),
						getLocation(),
						CallType.STATIC,
						null,
						"__rmod__",
						LeftToRightEvaluation.INSTANCE,
						getRight(),
						getLeft());
				try {
					interprocedural.resolve(rmod,
							new Set[] { Collections.singleton(tr), Collections.singleton(tl) }, aliasing);
					result = result.lub(rmod.forwardSemantics(state, interprocedural, expressions));
				} catch (CallResolutionException e) {
					// neither type implements %: this pair does not contribute to the result
				}
			}
		}

		return result;
	}
}
