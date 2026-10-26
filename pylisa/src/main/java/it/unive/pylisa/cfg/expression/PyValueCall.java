package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Arrays;
import java.util.Collection;
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

	/**
	 * Calls the given values of a callee on the given values of the arguments,
	 * as done by a call {@code f(a0, ..., an)} at {@code pp} whose callee and
	 * arguments have already been evaluated (e.g. the function passed to a
	 * native, as in {@code functools.reduce(f, xs)}): the targets are the same
	 * (see {@link CallTargets}), {@code TypeError} is raised if the callee
	 * might not be callable, and the result is unknown if its targets might not
	 * be known. The call happens at a location derived from the one of
	 * {@code pp} with the given name (see {@link DerivedLocation}), so that its
	 * result does not overwrite the one of {@code pp}.
	 *
	 * @param <A>             the kind of the analysis state
	 * @param <D>             the kind of the abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the callee and the arguments have
	 *                            been evaluated
	 * @param callee          the values of the callee
	 * @param args            the values of the arguments
	 * @param pp              the statement making the call
	 * @param name            the name of the call
	 * @param expressions     the states of the sub-expressions
	 * @param metas           where the meta variables of the call are added
	 *
	 * @return the state after the call
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> call(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet callee,
			ExpressionSet[] args,
			Statement pp,
			String name,
			StatementStore<A> expressions,
			Collection<Identifier> metas)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CFG cfg = pp.getCFG();
		CodeLocation loc = new DerivedLocation(pp.getLocation(), name);
		// the callee and the arguments are not expressions of the program:
		// they stand for their values, in the store of the sub-expressions
		Expression calleeExpr = new VariableRef(cfg, new DerivedLocation(loc, "callee"), "$callee");
		expressions.put(calleeExpr, state.withExecutionExpressions(callee));
		Expression[] actuals = new Expression[args.length];
		for (int i = 0; i < args.length; i++) {
			actuals[i] = new VariableRef(cfg, new DerivedLocation(loc, "arg" + i), "$arg" + i);
			expressions.put(actuals[i], state.withExecutionExpressions(args[i]));
		}
		UnresolvedCall call = new UnresolvedCall(cfg, loc, CallType.STATIC, null, name,
				LeftToRightEvaluation.INSTANCE, actuals);
		call.setParentStatement(pp);

		CallTargets targets = CallTargets.of(analysis, state, callee, true, pp);
		AnalysisState<A> result = targets.call(interprocedural, state, state, pp, call, calleeExpr, args, expressions,
				metas);
		if (targets.notCallable())
			result = result.lub(PyExceptions.raise(analysis, state, cfg, pp.getLocation(), pp,
					LibrarySpecificationProvider.TYPE_ERROR));
		if (targets.unknown())
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), pp));
		return result;
	}

	/**
	 * Whether the given value can be called (see {@code CallTargets}), as
	 * {@code callable(value)}.
	 *
	 * @param <A>      the kind of the analysis state
	 * @param <D>      the kind of the abstract domain
	 * @param analysis the analysis
	 * @param state    the state where the value has been computed
	 * @param value    the value
	 * @param pp       the program point using the value
	 *
	 * @return whether the value can be called
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability callable(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value,
			Statement pp)
			throws SemanticException {
		return CallTargets.callable(analysis.getRuntimeTypesOf(state, value, pp), pp);
	}
}
