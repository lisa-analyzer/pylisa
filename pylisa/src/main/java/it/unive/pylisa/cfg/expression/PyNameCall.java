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
import java.util.List;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.program.annotations.values.StringAnnotationValue;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.HashSet;
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

	/**
	 * Builds the call.
	 *
	 * @param cfg        the cfg where the call happens
	 * @param location   the location of the call
	 * @param name       the name being called
	 * @param order      the evaluation order of the parameters
	 * @param parameters the parameters of the call
	 */
	// the expression standing for the object of a bound method held by the
	// variable (see BoundMethods), that is not part of the cfg
	private final VariableRef boundObject;

	public PyNameCall(
			CFG cfg,
			CodeLocation location,
			String name,
			EvaluationOrder order,
			Expression... parameters) {
		super(cfg, location, CallType.STATIC, null, name, order, parameters);
		this.boundObject = new VariableRef(cfg, new DerivedLocation(location, "bound object"), name);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Variable callee = new Variable(Untyped.INSTANCE, getTargetName(), getLocation());
		if (!state.knowsIdentifier(callee))
			// e.g. a function of the module, called inside another function
			return byName(interprocedural, state, params, expressions);

		Set<CFG> cfgs = new HashSet<>();
		Set<NativeCFG> natives = new HashSet<>();
		boolean others = false, notCallable = false, bound = false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, callee, this);
		if (PyBinaryDispatch.unknownTypes(types, this))
			// e.g. a library function (as in f = len), whose type is unknown
			return byName(interprocedural, state, params, expressions);
		for (Type t : types)
			if (BoundMethods.isBoundMethod(t))
				bound = true;
			else if (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
				// e.g. an int, that cannot be called
				notCallable = true;
			else if (t.isCodeMemberType())
				for (CodeMember member : t.asCodeMemberType().getMembers())
					if (member instanceof CFG)
						cfgs.add((CFG) member);
					else if (member instanceof NativeCFG)
						natives.add((NativeCFG) member);
					else
						others = true;
			else
				others = true;

		if (cfgs.isEmpty() && natives.isEmpty() && !notCallable && !bound)
			// not a variable holding functions
			return byName(interprocedural, state, params, expressions);

		AnalysisState<A> result = state.bottomExecution();
		if (notCallable)
			result = result.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR));
		if (!cfgs.isEmpty())
			result = result.lub(call(CallSites.within(new CFGCall(this, cfgs), this), interprocedural, state, params, expressions));
		if (!natives.isEmpty())
			result = result.lub(call(CallSites.within(new NativeCall(this, natives), this), interprocedural, state, params, expressions));
		if (bound)
			// a bound method, called on its object (this call stands for it)
			result = result.lub(BoundMethods.call(interprocedural, state, callee, boundObject, getParameters(), params,
					this, expressions, getMetaVariables()));
		if (others)
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

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> call(
			Call call,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = call.forwardSemanticsAux(interprocedural, state, params, expressions);
		getMetaVariables().addAll(call.getMetaVariables());
		return result;
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
