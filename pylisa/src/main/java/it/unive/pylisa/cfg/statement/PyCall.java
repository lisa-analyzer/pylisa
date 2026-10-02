package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import java.util.Arrays;
import java.util.Set;

/**
 * A Python call, {@code callee(arguments)}, named after Python's
 * {@code ast.Call}. The callee is the first sub-expression; for a method call
 * written {@code receiver.attribute(arguments)}, the receiver is the second.
 * <p>
 * The call is resolved by the call graph of the analysis, as LiSA resolves
 * every call, from the runtime types of its operands alone, which this call
 * reads in the state it is applied to (see {@link #parameterTypes}). What it
 * resolves to, a {@link PyResolvedCall}, applies the targets; pylisa's call
 * graph is the one that dispatches Python calls, and an analysis with another
 * call graph stops at the first Python call.
 * </p>
 */
public class PyCall extends UnresolvedCall {

	private final boolean hasReceiver;

	/**
	 * Whether the frontend built this call to apply a decorator: when the
	 * decorator cannot be resolved, the call is assumed to return the
	 * decorated function, so that an outer decorator still sees it.
	 */
	private final boolean decoratorApplication;

	/**
	 * Builds a call with no receiver.
	 *
	 * @param cfg       the CFG the call belongs to
	 * @param location  the location of the call
	 * @param callee    the callee
	 * @param arguments the arguments
	 */
	public PyCall(
			CFG cfg,
			CodeLocation location,
			Expression callee,
			Expression[] arguments) {
		this(cfg, location, callee, arguments, false, false);
	}

	/**
	 * Builds a call.
	 *
	 * @param cfg         the CFG the call belongs to
	 * @param location    the location of the call
	 * @param callee      the callee
	 * @param arguments   the arguments, the receiver first if there is one
	 * @param hasReceiver whether the call is written
	 *                        {@code receiver.attribute(arguments)}
	 */
	public PyCall(
			CFG cfg,
			CodeLocation location,
			Expression callee,
			Expression[] arguments,
			boolean hasReceiver) {
		this(cfg, location, callee, arguments, hasReceiver, false);
	}

	/**
	 * Builds a call.
	 *
	 * @param cfg                  the CFG the call belongs to
	 * @param location             the location of the call
	 * @param callee               the callee
	 * @param arguments            the arguments, the receiver first if there
	 *                                 is one
	 * @param hasReceiver          whether the call is written
	 *                                 {@code receiver.attribute(arguments)}
	 * @param decoratorApplication whether the call applies a decorator
	 */
	public PyCall(
			CFG cfg,
			CodeLocation location,
			Expression callee,
			Expression[] arguments,
			boolean hasReceiver,
			boolean decoratorApplication) {
		super(cfg, location, CallType.UNKNOWN, "", "__call__", LeftToRightEvaluation.INSTANCE, Untyped.INSTANCE,
				prependCallee(arguments, callee));
		this.hasReceiver = hasReceiver;
		this.decoratorApplication = decoratorApplication;
	}

	/**
	 * Yields whether the frontend built this call to apply a decorator.
	 *
	 * @return {@code true} if it did
	 */
	public boolean isDecoratorApplication() {
		return decoratorApplication;
	}

	/**
	 * Yields whether the call is written {@code receiver.attribute(arguments)}.
	 *
	 * @return {@code true} if it is
	 */
	public boolean hasReceiver() {
		return hasReceiver;
	}

	@Override
	public String toString() {
		return getSubExpressions()[0] + "("
				+ Arrays.stream(getSubExpressions()).toList().subList(1, getSubExpressions().length) + ")";
	}

	private static Expression[] prependCallee(
			Expression[] arguments,
			Expression callee) {
		Expression[] result = new Expression[arguments.length + 1];
		result[0] = callee;
		System.arraycopy(arguments, 0, result, 1, arguments.length);
		return result;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		PyCall other = (PyCall) o;
		int cmp = compareCallAux(other);
		if (cmp != 0)
			return cmp;
		cmp = Integer.compare(getSubExpressions().length, other.getSubExpressions().length);
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < getSubExpressions().length; i++) {
			cmp = getSubExpressions()[i].toString().compareTo(other.getSubExpressions()[i].toString());
			if (cmp != 0)
				return cmp;
		}
		// two calls at the same location with identical sub-expressions are
		// the same call site: an identity tiebreaker would make the calls
		// built while analysing (e.g. by an instantiation) distinct sites on
		// every fixpoint iteration, preventing convergence
		return 0;
	}

	@Override
	protected int compareCallAux(
			Call o) {
		PyCall other = (PyCall) o;
		int cmp = Boolean.compare(hasReceiver, other.hasReceiver);
		if (cmp != 0)
			return cmp;
		return Boolean.compare(decoratorApplication, other.decoratorApplication);
	}

	/**
	 * Yields the runtime types of the operands, as the call graph dispatches
	 * on them. The callee's and the receiver's are read in the state the call
	 * is applied to, after every operand, since a later operand may rebind
	 * them: the callee's through {@link CallTargets#calleeTypes}, the
	 * receiver's through {@link CallTargets#receiverTypes}. The other
	 * operands' are LiSA's.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<Type>[] parameterTypes(
			StatementStore<A> expressions,
			Analysis<A, D> analysis)
			throws SemanticException {
		Set<Type>[] types = super.parameterTypes(expressions, analysis);
		Expression[] sub = getSubExpressions();
		AnalysisState<A> applied = expressions.getState(sub[sub.length - 1]);
		types[0] = CallTargets.calleeTypes(analysis, applied, expressions.getState(sub[0]).getExecutionExpressions(),
				this);
		if (receiverOperand())
			types[1] = CallTargets.receiverTypes(analysis, applied,
					expressions.getState(sub[1]).getExecutionExpressions(), this);
		return types;
	}

	/**
	 * Resolves this call through the call graph of the analysis and applies
	 * what it resolves to, as LiSA does for every call.
	 *
	 * @throws SemanticException if the call graph does not dispatch Python
	 *                               calls
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		PyResolvedCall resolved = resolve(interprocedural, state, expressions);
		AnalysisState<A> result = resolved.forwardSemanticsAux(interprocedural, state, params, expressions);
		getMetaVariables().addAll(resolved.getMetaVariables());
		return result;
	}

	/**
	 * Resolves this call through the call graph of the analysis, from the
	 * operand types stored in {@code expressions}.
	 *
	 * @param <A>             the kind of abstract state
	 * @param <D>             the kind of abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state the call is applied to
	 * @param expressions     the states after each operand
	 *
	 * @return what the call resolves to
	 *
	 * @throws SemanticException if the call graph does not dispatch Python
	 *                               calls
	 */
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> PyResolvedCall resolve(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			StatementStore<A> expressions)
			throws SemanticException {
		Set<Type>[] types = parameterTypes(expressions, interprocedural.getAnalysis());
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);
		Call resolved;
		try {
			resolved = interprocedural.resolve(this, types, aliasing);
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve " + this + " at " + getLocation(), e);
		}
		if (!(resolved instanceof PyResolvedCall python))
			throw new SemanticException(this + " at " + getLocation()
					+ ": Python calls need pylisa's call graph, got " + resolved.getClass().getSimpleName());
		// the resolution is cached by equal calls: an equal call built
		// elsewhere may have been resolved first
		if (!equals(resolved.getSource()))
			throw new SemanticException(this + " at " + getLocation() + " was resolved as another call, "
					+ resolved.getSource());
		return python;
	}

	/**
	 * Yields the arguments this call passes to the callables it calls: its
	 * sub-expressions after the callee, the receiver of a method call
	 * included, unless every value of the receiver is a module, as in
	 * {@code os.getcwd()}: a function reached through a module is not a
	 * method, and is not passed the module.
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param analysis  the analysis
	 * @param state     the state after the evaluation of the sub-expressions
	 * @param receivers the values of the receiver, the first sub-expression
	 *                      after the callee
	 *
	 * @return the arguments
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Expression[] arguments(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet receivers)
			throws SemanticException {
		return arguments(!receiverOperand()
				|| CallTargets.receiverPassed(CallTargets.receiverTypes(analysis, state, receivers, this)));
	}

	/**
	 * Yields the arguments this call passes to the callables it calls, as
	 * {@link #arguments(Analysis, AnalysisState, ExpressionSet)} does, from
	 * the runtime types of the operands given by {@link #parameterTypes}.
	 *
	 * @param types the runtime types of the operands
	 *
	 * @return the arguments
	 */
	Expression[] arguments(
			Set<Type>[] types) {
		return arguments(!receiverOperand() || CallTargets.receiverPassed(types[1]));
	}

	/**
	 * Yields whether the second sub-expression is a receiver, which the
	 * callables are not passed when every value of it is a module.
	 */
	private boolean receiverOperand() {
		return hasReceiver && getSubExpressions().length > 1;
	}

	private Expression[] arguments(
			boolean receiverPassed) {
		return Arrays.copyOfRange(getSubExpressions(), receiverPassed ? 1 : 2, getSubExpressions().length);
	}

	/**
	 * Yields the arguments this call passes to the constructor of a class it
	 * instantiates: its sub-expressions after the callee, without the receiver
	 * through which the class is reached, as in {@code module.Class(x)}.
	 *
	 * @return the arguments
	 */
	Expression[] constructorArguments() {
		int first = hasReceiver && getSubExpressions().length > 1 ? 2 : 1;
		return Arrays.copyOfRange(getSubExpressions(), first, getSubExpressions().length);
	}

	/**
	 * Yields the operands of an instantiation this call performs: the class
	 * expression, then the constructor arguments.
	 *
	 * @return the operands
	 */
	Expression[] instantiationOperands() {
		Expression[] arguments = constructorArguments();
		Expression[] operands = new Expression[arguments.length + 1];
		operands[0] = getSubExpressions()[0];
		System.arraycopy(arguments, 0, operands, 1, arguments.length);
		return operands;
	}
}
