package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.loader.LibraryNativeCFG;
import it.unive.pylisa.program.language.parameterassignment.ArgumentBinding;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The rule that dispatches a Python call, shared by the analysis and by the
 * readers of its results. The targets of a call depend on the runtime types
 * of its callee alone (see {@link #calleeTypes} and {@link #targets}): every
 * type either yields a target or an {@link Unresolved} part, so that no
 * execution of the call is silently dropped. The analysis continues
 * unresolved parts with an unknown result, and readers of the results see
 * them.
 */
public final class CallTargets {

	/**
	 * Beyond this many runtime types, a callee is not dispatched type by type.
	 */
	private static final int TYPE_LIMIT = 20;

	/**
	 * A target of a call.
	 */
	public sealed interface Target permits Native, Python, Instantiation, Unresolved {
	}

	/**
	 * A callable declared in a library specification.
	 *
	 * @param cfg            the code of the callable
	 * @param implementation the class modelling the callable
	 * @param library        the library declaring the callable
	 */
	public record Native(LibraryNativeCFG cfg, Class<? extends NaryExpression> implementation, String library)
			implements
			Target {
	}

	/**
	 * A function whose Python code is analysed.
	 *
	 * @param cfg the code of the function
	 */
	public record Python(CFG cfg) implements Target {
	}

	/**
	 * The construction of an instance of a class. Its parts, {@code __new__}
	 * and {@code __init__}, depend on the state the class is instantiated in,
	 * not only on the type of the callee: see {@link #constructorParts}.
	 *
	 * @param type the class
	 */
	public record Instantiation(PyClassType type) implements Target {
	}

	/**
	 * The parts of the construction of an instance of a class, as resolved in
	 * one state, with an unresolved part for each part of {@code __new__} or
	 * {@code __init__} that cannot be dispatched.
	 *
	 * @param creation       the targets of {@code __new__}
	 * @param initialization the targets of {@code __init__}
	 */
	public record ConstructorParts(List<Target> creation, List<Target> initialization) {
	}

	/**
	 * A part of the call that cannot be dispatched: the analysis continues it
	 * with an unknown result.
	 *
	 * @param reason why the part cannot be dispatched
	 */
	public record Unresolved(String reason) implements Target {
	}

	private CallTargets() {
	}

	/**
	 * Yields the targets of a call, as stored in the results of the analysis
	 * of its CFG: the callee is read after the evaluation of the callee
	 * expression, and its types in the state after all the sub-expressions,
	 * which is the state the call is applied to.
	 *
	 * @param <A>      the kind of abstract state
	 * @param <D>      the kind of abstract domain
	 * @param analysis the analysis
	 * @param result   the results of the CFG of the call, in one context
	 * @param call     the call
	 *
	 * @return the targets; empty if no execution applies the call
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> List<Target> of(
			Analysis<A, D> analysis,
			AnalyzedCFG<A> result,
			PyCall call)
			throws SemanticException {
		Expression[] sub = call.getSubExpressions();
		AnalysisState<A> applied = result.getAnalysisStateAfter(sub[sub.length - 1]);
		if (applied.getExecution().isBottom() || applied.getExecutionState().isBottom())
			return List.of();
		ExpressionSet callees = result.getAnalysisStateAfter(sub[0]).getExecutionExpressions();
		return targets(calleeTypes(analysis, applied, callees, call));
	}

	/**
	 * The arguments of a call bound to one formal parameter of a callee.
	 *
	 * @param formal    the formal parameter
	 * @param arguments the arguments bound to it: one for a plain parameter,
	 *                      any number for a {@code *args} or {@code **kw}
	 *                      parameter, none for a parameter that takes its
	 *                      default
	 */
	public record Binding(Parameter formal, List<Expression> arguments) {
	}

	/**
	 * Binds the arguments of a call to the formal parameters of one of its
	 * targets, as stored in the results of the analysis of its CFG and as the
	 * dispatch passes them (see {@link ArgumentBinding}): a callable receives
	 * {@link PyCall#arguments}; the {@code __init__} of an
	 * instantiation receives the object being created and then the
	 * constructor arguments, and its first parameter, bound to the object, is
	 * left out.
	 *
	 * @param <A>            the kind of abstract state
	 * @param <D>            the kind of abstract domain
	 * @param analysis       the analysis
	 * @param result         the results of the CFG of the call, in one context
	 * @param call           the call, applied in that context
	 * @param callee         the target, or the {@code __init__} of an
	 *                           instantiation
	 * @param initialization whether {@code callee} is the {@code __init__} of
	 *                           an instantiation
	 *
	 * @return the binding of each formal parameter, in order; empty when the
	 *             arguments do not match the parameters
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Optional<List<Binding>> bind(
			Analysis<A, D> analysis,
			AnalyzedCFG<A> result,
			PyCall call,
			CodeMember callee,
			boolean initialization)
			throws SemanticException {
		Parameter[] formals = callee.getDescriptor().getFormals();
		Expression[] actuals;
		if (initialization) {
			if (formals.length == 0)
				return Optional.empty();
			formals = Arrays.copyOfRange(formals, 1, formals.length);
			actuals = call.constructorArguments();
		} else {
			Expression[] sub = call.getSubExpressions();
			actuals = sub.length < 2 ? new Expression[0]
					: call.arguments(analysis, result.getAnalysisStateAfter(sub[sub.length - 1]),
							result.getAnalysisStateAfter(sub[1]).getExecutionExpressions());
		}
		Optional<List<List<Integer>>> positions = ArgumentBinding.bind(formals, actuals);
		if (positions.isEmpty())
			return Optional.empty();
		List<Binding> bindings = new ArrayList<>();
		for (int i = 0; i < formals.length; i++)
			bindings.add(new Binding(formals[i],
					positions.get().get(i).stream().map(position -> actuals[position]).toList()));
		return Optional.of(bindings);
	}

	/**
	 * Yields the runtime types of the callee of a call, in the state the call
	 * is applied to. {@link NoInfoType} stands for every part of the callee
	 * that cannot be dispatched by type: an unknown callee, a callee whose
	 * type the analysis does not know (a library global referred to by its
	 * name also gets its registered types), and the types of a callee
	 * expression beyond {@value #TYPE_LIMIT}. The limit applies to each callee
	 * expression on its own, counting all its types: the types of several
	 * callee expressions are dispatched together, whatever their total. The
	 * types keep the order of the callee expressions, so that the targets are
	 * always dispatched in the same order.
	 *
	 * @param <A>      the kind of abstract state
	 * @param <D>      the kind of abstract domain
	 * @param analysis the analysis
	 * @param state    the state after the evaluation of every sub-expression
	 *                     of the call
	 * @param callees  the values of the callee expression
	 * @param point    the program point of the call
	 *
	 * @return the types; empty if the callee has no value
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<Type> calleeTypes(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet callees,
			ProgramPoint point)
			throws SemanticException {
		Set<Type> types = new LinkedHashSet<>();
		for (SymbolicExpression callee : callees)
			types.addAll(typesOfCallee(analysis, state, callee, point));
		return types;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<Type> typesOfCallee(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression callee,
			ProgramPoint point)
			throws SemanticException {
		if (callee instanceof PushAny)
			return Set.of(NoInfoType.INSTANCE);
		Set<Type> types = new LinkedHashSet<>(analysis.getRuntimeTypesOf(state, callee, point));
		boolean untyped = types.isEmpty() || types.stream().allMatch(NoInfoType.INSTANCE::equals);
		if (untyped) {
			// the state may not carry the type of a library global referred
			// to by its qualified name: the registered type is used, but the
			// analysis did not derive it
			types = new LinkedHashSet<>();
			if (callee instanceof GlobalVariable global)
				types.addAll(registeredTypes(global.getName()));
		}
		if (types.size() > TYPE_LIMIT) {
			Set<Type> callable = new LinkedHashSet<>();
			for (Type type : types)
				if (type instanceof PyFunctionType || type instanceof PyClassType)
					callable.add(type);
			if (callable.isEmpty() || callable.size() > TYPE_LIMIT)
				return Set.of(NoInfoType.INSTANCE);
			types = callable;
			types.add(NoInfoType.INSTANCE);
		}
		if (untyped)
			types.add(NoInfoType.INSTANCE);
		return types;
	}

	/**
	 * Yields the targets of a call from the runtime types of its callee, as
	 * {@link #calleeTypes} gives them: a function or a library model for each
	 * function type, an instantiation for each class, and an unresolved part
	 * for {@link NoInfoType} and for each type that is not callable.
	 *
	 * @param calleeTypes the types of the callee
	 *
	 * @return the targets; an unresolved part alone if the callee has no value
	 */
	public static List<Target> targets(
			Set<Type> calleeTypes) {
		if (calleeTypes.isEmpty())
			return List.of(new Unresolved("the callee has no value"));
		List<Target> targets = new ArrayList<>();
		for (Type type : calleeTypes)
			if (type instanceof PyClassType classType)
				targets.add(new Instantiation(classType));
			else if (type instanceof PyFunctionType function)
				targets.add(function(function));
			else if (NoInfoType.INSTANCE.equals(type))
				targets.add(new Unresolved("the callee may have a type the analysis does not know"));
			else
				targets.add(new Unresolved("the callee may be a " + describe(type) + ", which is not dispatched"));
		return targets;
	}

	/**
	 * Yields the runtime types of the receiver of a method call, in the state
	 * the call is applied to. A value of the receiver whose type the analysis
	 * does not know contributes {@link NoInfoType}, so that it is never taken
	 * for a module (see {@link #receiverPassed}).
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param analysis  the analysis
	 * @param state     the state after the evaluation of every sub-expression
	 *                      of the call
	 * @param receivers the values of the receiver expression
	 * @param point     the program point of the call
	 *
	 * @return the types; empty if the receiver has no value
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<Type> receiverTypes(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet receivers,
			ProgramPoint point)
			throws SemanticException {
		Set<Type> types = new LinkedHashSet<>();
		for (SymbolicExpression receiver : receivers) {
			Set<Type> of = analysis.getRuntimeTypesOf(state, receiver, point);
			if (of.isEmpty())
				types.add(NoInfoType.INSTANCE);
			else
				types.addAll(of);
		}
		return types;
	}

	/**
	 * Yields whether the receiver of a method call is passed to the callables
	 * it calls. It is, unless every value of the receiver is a module, as in
	 * {@code os.getcwd()}: a function reached through a module is not a
	 * method, and is not passed the module.
	 *
	 * @param receiverTypes the types of the receiver, as
	 *                          {@link #receiverTypes} gives them
	 *
	 * @return whether the receiver is passed
	 */
	public static boolean receiverPassed(
			Set<Type> receiverTypes) {
		return receiverTypes.isEmpty() || !receiverTypes.stream().allMatch(PyModuleType.class::isInstance);
	}

	/**
	 * Yields the parts of the construction of an instance of a class in a
	 * state: the targets of {@code __new__} and of {@code __init__}, as
	 * {@link #attribute} resolves them in that state.
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param analysis  the analysis
	 * @param state     the state the class is instantiated in
	 * @param classType the class
	 * @param point     the program point of the instantiation
	 *
	 * @return the parts
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ConstructorParts constructorParts(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			PyClassType classType,
			ProgramPoint point)
			throws SemanticException {
		return new ConstructorParts(
				methodTargets(attribute(analysis, state, classType, "__new__", point), "__new__", classType),
				methodTargets(attribute(analysis, state, classType, "__init__", point), "__init__", classType));
	}

	private static Set<Type> registeredTypes(
			String name) {
		String qualified = name.startsWith("$") ? name.substring(1).replace("::", ".") : name;
		if (PyFunctionType.isRegistered(qualified))
			return Set.of(PyFunctionType.lookup(qualified));
		// conditional class redefinitions share a qualified name: all of them
		// are candidates
		return new LinkedHashSet<>(PyClassType.lookupAllByBaseName(qualified));
	}

	private static String describe(
			Type type) {
		return type instanceof ReferenceType reference ? "reference to " + reference.getInnerType() : type.toString();
	}

	/**
	 * Yields the target of calling a function.
	 *
	 * @param function the type of the function
	 *
	 * @return the target
	 */
	static Target function(
			PyFunctionType function) {
		CodeMember code = function.getUnit().getFunction();
		if (code instanceof LibraryNativeCFG cfg)
			return new Native(cfg, cfg.getImplementation(), cfg.getLibrary());
		if (code instanceof CFG cfg)
			return new Python(cfg);
		return new Unresolved("the function " + function + " has no code");
	}

	/**
	 * Yields the targets of a method of a class, as resolved by
	 * {@link #attribute}: a function target for each function, and an
	 * unresolved part for a method that is not found, found only by its name,
	 * or that may not be a function.
	 *
	 * @param resolution the resolution of the method
	 * @param method     the name of the method
	 * @param classType  the class
	 *
	 * @return the targets
	 */
	static List<Target> methodTargets(
			Resolution resolution,
			String method,
			PyClassType classType) {
		List<Target> targets = new ArrayList<>();
		if (resolution.types().isEmpty())
			targets.add(new Unresolved("no " + method + " of " + classType + " is found (" + resolution.mode() + ")"));
		if (resolution.mode().endsWith("-registry"))
			targets.add(new Unresolved(method + " of " + classType + " is found only by its name"));
		for (Type type : resolution.types())
			if (type instanceof PyFunctionType function)
				targets.add(function(function));
			else if (NoInfoType.INSTANCE.equals(type))
				targets.add(
						new Unresolved(method + " of " + classType + " may have a type the analysis does not know"));
			else
				targets.add(new Unresolved(method + " of " + classType + " may be a " + describe(type)));
		return targets;
	}

	/**
	 * The resolution of an attribute of a class through its ancestors.
	 *
	 * @param owner      the class defining the attribute, or {@code null} if
	 *                       none does
	 * @param types      the runtime types of the attribute, empty if it is not
	 *                       found
	 * @param lookupPath the classes visited, in order
	 * @param mode       how the attribute was found
	 */
	record Resolution(CompilationUnit owner, Set<Type> types, String lookupPath, String mode) {
	}

	/**
	 * Resolves an attribute of a class as Python does for a single chain of
	 * ancestors: the first class defining it wins, whether or not its value is
	 * callable, and all the types of its value are kept, including a type the
	 * analysis does not know. Classes with more than one direct ancestor are
	 * not supported, and their attributes are not found.
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param analysis  the analysis
	 * @param state     the state the attribute is read in
	 * @param classType the class
	 * @param attribute the name of the attribute
	 * @param point     the program point reading it
	 *
	 * @return the resolution
	 *
	 * @throws SemanticException if the types cannot be computed
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Resolution attribute(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			PyClassType classType,
			String attribute,
			ProgramPoint point)
			throws SemanticException {
		LinkedHashSet<String> visited = new LinkedHashSet<>();
		ArrayDeque<CompilationUnit> work = new ArrayDeque<>();
		work.add(classType.getUnit());
		while (!work.isEmpty()) {
			CompilationUnit current = work.removeFirst();
			if (!visited.add(current.getName()))
				continue;
			GlobalVariable variable = new GlobalVariable(Untyped.INSTANCE,
					"$" + current.getName() + "::" + attribute, point.getLocation());
			Set<Type> types = analysis.getRuntimeTypesOf(state, variable, point);
			String inherited = visited.size() == 1 ? "direct" : "inherited";
			if (types.stream().anyMatch(type -> !NoInfoType.INSTANCE.equals(type)))
				return new Resolution(current, types, String.join(" -> ", visited), inherited);
			// the state may not carry the binding of a library method in deep
			// contexts: a library method has a stable qualified name
			String qualified = current.getName() + "." + attribute;
			if (PyFunctionType.isRegistered(qualified))
				return new Resolution(current, Set.of(PyFunctionType.lookup(qualified)),
						String.join(" -> ", visited), inherited + "-registry");
			Collection<CompilationUnit> ancestors = current.getImmediateAncestors();
			if (ancestors.size() > 1)
				return new Resolution(null, Set.of(), String.join(" -> ", visited), "unsupported-multiple-ancestors");
			work.addAll(ancestors);
		}
		return new Resolution(null, Set.of(), String.join(" -> ", visited), "unresolved");
	}
}
