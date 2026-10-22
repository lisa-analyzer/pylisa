package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * A call whose callee is the value of an expression that is not a name, e.g.
 * {@code d[key]()}, {@code f()()} or {@code o.m()()}. The callee is evaluated
 * first, then the arguments, and the call is resolved on the values of the
 * callee:
 * <ul>
 * <li>functions (e.g. the ones defined by {@code def}) are called;</li>
 * <li>{@code None}, numbers, strings and the objects of classes that do not
 * define {@code __call__} cannot be called, and raise {@code TypeError};</li>
 * <li>for any other value (e.g. a class, or an unknown value), the result is
 * unknown.</li>
 * </ul>
 */
public class PyValueCall extends NaryExpression {

	/**
	 * Builds the call.
	 *
	 * @param cfg       the cfg where the call is
	 * @param location  the location of the call
	 * @param callee    the expression computing the called value
	 * @param arguments the arguments of the call
	 */
	public PyValueCall(
			CFG cfg,
			CodeLocation location,
			Expression callee,
			Expression... arguments) {
		super(cfg, location, "()", LeftToRightEvaluation.INSTANCE, concat(callee, arguments));
	}

	private static Expression[] concat(
			Expression callee,
			Expression[] arguments) {
		Expression[] subs = new Expression[arguments.length + 1];
		subs[0] = callee;
		System.arraycopy(arguments, 0, subs, 1, arguments.length);
		return subs;
	}

	/**
	 * Yields the expression computing the called value.
	 *
	 * @return the callee
	 */
	public Expression getCallee() {
		return getSubExpressions()[0];
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		Expression[] subs = getSubExpressions();
		StringBuilder res = new StringBuilder(subs[0].toString()).append("(");
		for (int i = 1; i < subs.length; i++)
			res.append(i > 1 ? ", " : "").append(subs[i]);
		return res.append(")").toString();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Expression[] subs = getSubExpressions();
		// the callee is called without itself as argument
		UnresolvedCall call = new UnresolvedCall(getCFG(), getLocation(), CallType.STATIC, null, "()",
				getOrder(), Arrays.copyOfRange(subs, 1, subs.length));
		ExpressionSet[] args = Arrays.copyOfRange(params, 1, params.length);
		// the value of the callee is consumed here
		AnalysisState<A> consumed = state.forgetIdentifiers(subs[0].getMetaVariables(), this);

		CallTargets targets = CallTargets.of(analysis, state, params[0], true, this);
		AnalysisState<A> result = state.bottomExecution();
		if (targets.notCallable())
			result = result.lub(PyExceptions.raise(analysis, consumed, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR));
		// closures receive the value of the callee as their first argument:
		// it is consumed after the call (unless it is also the result of this
		// call, e.g. in f()() where the two calls have the same location)
		AnalysisState<A> called = targets.call(interprocedural, consumed, state, this, call, subs[0], args, expressions,
				getMetaVariables());
		Set<Identifier> calleeValues = new HashSet<>(subs[0].getMetaVariables());
		calleeValues.removeAll(getMetaVariables());
		result = result.lub(called.forgetIdentifiers(calleeValues, this));
		if (targets.unknown())
			result = result.lub(analysis.smallStepSemantics(consumed, new PushAny(Untyped.INSTANCE, getLocation()),
					this));
		return result;
	}
}
