package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.values.StringAnnotationValue;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.namespaces.Binding;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.List;
import java.util.Set;

/**
 * A call by plain name ({@code f(args)}). If {@code f} is a variable holding
 * functions (i.e., whose runtime types are
 * {@link it.unive.lisa.type.CodeMemberType}s, e.g. after {@code f = greet}),
 * those functions are called; if it holds a value of a builtin type that cannot
 * be called (e.g. an {@code int}), {@code TypeError} is raised. Otherwise, the
 * call is resolved by name, as usual.
 */
public class PyNameCall extends UnresolvedCall {

	// the identifier the called name resolves to
	private final Identifier callee;

	// the reference to the called name, if it is a name of the program
	private final PyNameRef ref;

	/**
	 * Builds the call.
	 *
	 * @param cfg        the cfg where the call happens
	 * @param location   the location of the call
	 * @param name       the name being called
	 * @param callee     the identifier the called name resolves to (see
	 *                       {@link PyNameRef#getIdentifier()})
	 * @param ref        the reference to the called name, used for its binding
	 *                       status (see {@link PyNameRef#status}), or
	 *                       {@code null} if the name is always bound (e.g. a
	 *                       variable of a comprehension)
	 * @param order      the evaluation order of the parameters
	 * @param parameters the parameters of the call
	 */
	public PyNameCall(
			CFG cfg,
			CodeLocation location,
			String name,
			Identifier callee,
			PyNameRef ref,
			EvaluationOrder order,
			Expression... parameters) {
		super(cfg, location, CallType.STATIC, null, name, order, parameters);
		this.callee = callee;
		this.ref = ref;
	}

	/**
	 * Calls the name. If the name is bound, the value it holds is called (see
	 * {@link #boundCall}). If it is not bound, and it is a builtin (see
	 * {@link PyNameRef#isBuiltin()}), the call is resolved by name (e.g. to the
	 * native modeling the builtin), otherwise {@code NameError} or
	 * {@code UnboundLocalError} is raised.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Binding status = ref == null ? null : ref.status(state);
		if (status == null || status == Binding.BOTTOM || status == Binding.BOUND)
			return boundCall(interprocedural, state, params, expressions);

		AnalysisState<A> result = state.bottomExecution();
		if (status.mightBeBound())
			result = result.lub(boundCall(interprocedural, state, params, expressions));
		if (ref.isBuiltin())
			result = result.lub(super.forwardSemanticsAux(interprocedural, state, params, expressions));
		else
			result = result.lub(ref.unboundError(interprocedural.getAnalysis(), state));
		return result;
	}

	// calls the name, when it is bound: if its value is known, the functions
	// it holds are called, otherwise the call is resolved by name (see byName)
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> boundCall(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		if (!state.knowsIdentifier(callee))
			return byName(interprocedural, state, params, expressions);

		CallTargets targets = CallTargets.of(analysis, state, Set.of(callee), false, this);
		if (!targets.callsSomething() && !targets.notCallable())
			// not a variable holding functions (e.g. a library function, as
			// in f = len, whose type is unknown)
			return byName(interprocedural, state, params, expressions);

		// the callee is passed to closures, as their first argument
		Expression calleeExpr = ref != null ? ref : new VariableRef(getCFG(), getLocation(), getTargetName());
		expressions.put(calleeExpr, analysis.smallStepSemantics(state, callee, this));

		AnalysisState<A> result = state.bottomExecution();
		if (targets.notCallable())
			result = result.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR));
		result = result.lub(
				targets.call(interprocedural, state, state, this, this, calleeExpr, params, expressions,
						getMetaVariables()));
		if (targets.unknown())
			// the variable might also hold something else
			result = result.lub(byName(interprocedural, state, params, expressions));
		return result;
	}

	// the call of the function with the name of the call (e.g. a def of the
	// module): if it is the name of a decorated function, the name is bound to
	// the value returned by the decorator, that is not known here (e.g. inside
	// another function, or when the decorator is not known), and the call is
	// code that is not known
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> byName(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = super.forwardSemanticsAux(interprocedural, state, params, expressions);
		if (!decorated(getProgram(), getTargetName()))
			return result;
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		return result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, getLocation()), this))
				.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
						LibrarySpecificationProvider.EXCEPTION));
	}

	/**
	 * The annotation of the cfg of a decorated function (that is not named as
	 * the function, see {@code PyStatementParser#visitFunction_def}).
	 */
	public static final String DECORATED = "pylisa.decorated_function";

	/**
	 * Builds the annotation of the cfg of the decorated function with the given
	 * name.
	 *
	 * @param name the name of the function
	 *
	 * @return the annotation
	 */
	public static Annotation decorated(
			String name) {
		// not propagated by LiSA to overriding members
		return new Annotation(DECORATED, List.of(new AnnotationMember(DECORATED, new StringAnnotationValue(name))),
				true);
	}

	// whether the program has a decorated function with the given name
	private static boolean decorated(
			Program program,
			String name) {
		for (CFG cfg : program.getAllCFGs())
			for (Annotation ann : cfg.getDescriptor().getAnnotationList())
				if (ann.getAnnotationName().equals(DECORATED))
					for (AnnotationMember member : ann.getAnnotationMembers())
						// toString is the getter of the value
						if (member.getValue().toString().equals(name))
							return true;
		return false;
	}
}
