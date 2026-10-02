package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.ResolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.natives.LibraryNative;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;

/**
 * What a {@link PyCall} resolves to for one array of runtime types of its
 * operands: its targets (see {@link CallTargets#targets}), each applied by a
 * call built once, when the call is resolved. Its result is the join of the
 * results of the targets; an unresolved part gives an unknown result, except
 * for a decorator that cannot be resolved, which is assumed to return the
 * function it decorates.
 * <p>
 * An unresolved part does not change anything but the result: the unknown
 * code it stands for is not assumed to modify the heap or the globals it
 * could reach. The call graph reports every unresolved part, so that readers
 * of the results know about it.
 * </p>
 * <p>
 * The calls that apply the targets have the {@link PyCall} as parent and no
 * source, so that the errors they raise are raised by them, as for any call
 * of a library model or of a function, and they are evaluated, and named, as
 * the calls LiSA analyses for a function or a library model.
 * </p>
 */
public class PyResolvedCall extends Call implements ResolvedCall {

	private final PyCall site;

	private final List<CallTargets.Target> targets;

	/**
	 * How each target is applied, in the order of the targets: a call for a
	 * function or a library model, an instantiation for a class, nothing for
	 * an unresolved part.
	 */
	private final List<Expression> applications = new ArrayList<>();

	/**
	 * Resolves a call for the runtime types of its operands. Only the call
	 * graph builds resolved calls, once per call and array of types.
	 *
	 * @param site  the call
	 * @param types the runtime types of its operands, as
	 *                  {@link PyCall#parameterTypes} yields them
	 */
	public PyResolvedCall(
			PyCall site,
			Set<Type>[] types) {
		super(site.getCFG(), site.getLocation(), CallType.UNKNOWN, "", site.getTargetName(),
				LeftToRightEvaluation.INSTANCE, Untyped.INSTANCE, site.getSubExpressions());
		this.site = site;
		// the operands keep the call that is resolved as their parent, here and
		// in the calls applying the targets: LiSA sets the parent of an
		// expression only once
		this.targets = CallTargets.targets(types[0]);
		Expression[] arguments = site.arguments(types);
		for (CallTargets.Target target : targets)
			applications.add(application(target, arguments));
	}

	private Expression application(
			CallTargets.Target target,
			Expression[] arguments) {
		Expression application;
		if (target instanceof CallTargets.Instantiation instantiation)
			application = new PyInstantiation(site.getCFG(), site.getLocation(), instantiation.type(),
					site.instantiationOperands());
		else if (target instanceof CallTargets.Native natives)
			application = new NativeCall(site.getCFG(), site.getLocation(), Call.CallType.STATIC, "", "$call",
					List.of(natives.cfg()), arguments);
		else if (target instanceof CallTargets.Python python)
			application = new CFGCall(site.getCFG(), site.getLocation(), Call.CallType.STATIC, "", "$call",
					List.of(python.cfg()), arguments);
		else
			return null;
		application.setParentStatement(site);
		return application;
	}

	/**
	 * Yields the targets of the call, in dispatch order.
	 *
	 * @return the targets
	 */
	public List<CallTargets.Target> targets() {
		return targets;
	}

	/**
	 * Yields the parts of the call that cannot be dispatched.
	 *
	 * @return the unresolved parts; empty if the call is fully resolved
	 */
	public List<CallTargets.Unresolved> unresolved() {
		List<CallTargets.Unresolved> unresolved = new ArrayList<>();
		for (CallTargets.Target target : targets)
			if (target instanceof CallTargets.Unresolved part)
				unresolved.add(part);
		return unresolved;
	}

	/**
	 * Yields the calls that apply the function and library model targets, so
	 * that the call graph keeps them out of the call sites.
	 *
	 * @return the calls
	 */
	public List<Call> applications() {
		List<Call> calls = new ArrayList<>();
		for (Expression application : applications)
			if (application instanceof Call call)
				calls.add(call);
		return calls;
	}

	@Override
	public Collection<CodeMember> getTargets() {
		List<CodeMember> members = new ArrayList<>();
		for (CallTargets.Target target : targets)
			if (target instanceof CallTargets.Native natives)
				members.add(natives.cfg());
			else if (target instanceof CallTargets.Python python)
				members.add(python.cfg());
		return members;
	}

	@Override
	protected int compareCallAux(
			Call o) {
		return site.compareTo(((PyResolvedCall) o).site);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = state.bottomExecution();
		for (AnalysisState<A> each : applyEach(interprocedural, state, params, expressions, true))
			result = result.lub(each);
		return result;
	}

	/**
	 * Applies each target on its own, to the operands already evaluated.
	 *
	 * @param <A>             the kind of abstract state
	 * @param <D>             the kind of abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state the call is applied to
	 * @param params          the values of the operands of the call
	 * @param expressions     the states after each operand
	 * @param noResultUnknown whether a function or a native that is not a
	 *                            library model and leaves no result gives an
	 *                            unknown result, as for the value of a call;
	 *                            the creation of an object keeps the empty
	 *                            result, where no object is created yet
	 *
	 * @return the state after each target, in the order of the targets
	 *
	 * @throws SemanticException if a target cannot be applied
	 */
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> List<AnalysisState<A>> applyEach(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions,
			boolean noResultUnknown)
			throws SemanticException {
		List<AnalysisState<A>> results = new ArrayList<>();
		for (int i = 0; i < targets.size(); i++)
			results.add(apply(targets.get(i), applications.get(i), interprocedural, state, params, expressions,
					noResultUnknown));
		return results;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> apply(
			CallTargets.Target target,
			Expression application,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions,
			boolean noResultUnknown)
			throws SemanticException {
		if (application == null)
			return unknownResult(interprocedural, state, params);
		AnalysisState<A> result = application.forwardSemantics(state, interprocedural, expressions);
		if (application instanceof PyInstantiation)
			return result;
		// a library model is checked to have a continuation for every
		// reachable input unless its callable may never return, so its result
		// is kept as it is; any other callee with no result (a Python function
		// whose recursion is still being computed, a native that is not such a
		// model) is given an unknown one
		if (!noResultUnknown || !result.isBottom() || target instanceof CallTargets.Native natives
				&& LibraryNative.class.isAssignableFrom(natives.implementation()))
			return result;
		return unknownResult(interprocedural, state, params);
	}

	/**
	 * Yields the result of a part of the call that cannot be dispatched: an
	 * unknown value. A decorator that cannot be resolved is assumed to return
	 * the function it decorates, so that an outer decorator still sees it.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unknownResult(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params)
			throws SemanticException {
		if (site.isDecoratorApplication() && params.length > 1) {
			AnalysisState<A> decorated = state.bottomExecution();
			for (SymbolicExpression function : params[1])
				decorated = decorated.lub(interprocedural.getAnalysis().smallStepSemantics(state, function, site));
			return decorated;
		}
		return interprocedural.getAnalysis().smallStepSemantics(state,
				new PushAny(Untyped.INSTANCE, site.getLocation()), site);
	}
}
