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
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyClosureUnit;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import org.apache.commons.lang3.ArrayUtils;

/**
 * The targets of a call whose callee is a value (e.g. a variable holding a
 * function): the functions and natives it holds, the closures of nested
 * functions (see {@link PyClosureUnit}), the bound methods (see
 * {@link BoundMethods}), the objects whose class defines {@code __call__}, the
 * classes of the program, and whether it might hold values that cannot be
 * called, or whose callability is unknown. This is shared by the calls on names
 * ({@link PyNameCall}) and on other values ({@link PyValueCall}).
 */
final class CallTargets {

	// the functions and natives called directly
	private final Set<CFG> functions = new HashSet<>();

	private final Set<NativeCFG> natives = new HashSet<>();

	// the functions called through their closures, with the closures
	private final Map<String, Set<SymbolicExpression>> closures = new TreeMap<>();

	private final Map<String, CFG> closureFunctions = new TreeMap<>();

	// the bound methods, called on their objects
	private final Set<SymbolicExpression> boundMethods = new HashSet<>();

	// the objects whose class defines __call__, that is called on them
	private final Set<SymbolicExpression> callableObjects = new HashSet<>();

	// the classes of the program, whose objects are created by the call
	private final Map<String, PyClassType> classes = new TreeMap<>();

	// whether the callee might be a value that cannot be called
	private boolean notCallable = false;

	// whether the callee might be a value whose callability is unknown
	private boolean unknown = false;

	private CallTargets() {
	}

	/**
	 * Collects the targets of calling the given values.
	 *
	 * @param <A>                the kind of {@link AbstractLattice} of the
	 *                               state
	 * @param <D>                the kind of {@link AbstractDomain} of the
	 *                               analysis
	 * @param analysis           the analysis
	 * @param state              the state where the values are computed
	 * @param values             the values of the callee
	 * @param objectsNotCallable whether the objects of classes of the program
	 *                               that do not define {@code __call__} are
	 *                               considered not callable (otherwise, they
	 *                               are unknown)
	 * @param pp                 the program point of the call
	 *
	 * @return the targets
	 *
	 * @throws SemanticException if the analysis fails
	 */
	static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> CallTargets of(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Iterable<SymbolicExpression> values,
			boolean objectsNotCallable,
			Statement pp)
			throws SemanticException {
		CallTargets targets = new CallTargets();
		for (SymbolicExpression value : values) {
			Set<Type> types = analysis.getRuntimeTypesOf(state, value, pp);
			if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp)) {
				targets.unknown = true;
				continue;
			}
			for (Type t : types) {
				CFG closure = closureFunction(t);
				if (BoundMethods.isBoundMethod(t))
					targets.boundMethods.add(value);
				else if (closure != null) {
					String key = closure.getDescriptor().getFullSignature() + closure.getDescriptor().getLocation();
					targets.closureFunctions.put(key, closure);
					targets.closures.computeIfAbsent(key, k -> new HashSet<>()).add(value);
				} else if (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
					// e.g. an int, that cannot be called
					targets.notCallable = true;
				else if (isObject(t) && callable(t))
					// an object whose class defines __call__
					targets.callableObjects.add(value);
				else if (t.isTypeTokenType())
					// a class: its objects are created
					for (Type cls : t.asTypeTokenType().getTypes())
						if (isProgramClass(cls))
							targets.classes.put(cls.toString(), (PyClassType) cls);
						else
							targets.unknown = true;
				else if (objectsNotCallable && isObject(t) && PyBinaryDispatch.fullyKnown(t) && !callable(t))
					// an object whose class does not define __call__
					targets.notCallable = true;
				else if (t.isCodeMemberType())
					for (CodeMember member : t.asCodeMemberType().getMembers())
						if (member instanceof CFG)
							targets.functions.add((CFG) member);
						else if (member instanceof NativeCFG)
							targets.natives.add((NativeCFG) member);
						else
							targets.unknown = true;
				else
					// e.g. an object whose class might define __call__
					targets.unknown = true;
			}
		}
		return targets;
	}

	/**
	 * Whether the values of the given types can be called, as classified by
	 * {@link #of} (where the objects of the classes of the program that do not
	 * define {@code __call__} cannot be called): functions, natives, closures,
	 * bound methods, classes and the objects whose class defines
	 * {@code __call__} can be called, while the values of the builtin classes
	 * whose methods are all known, lists, dictionaries and tuples cannot.
	 *
	 * @param types the runtime types of the values
	 * @param pp    the statement using the values
	 *
	 * @return whether the values can be called
	 *
	 * @throws SemanticException if the analysis fails
	 */
	static Satisfiability callable(
			Set<Type> types,
			Statement pp)
			throws SemanticException {
		if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, pp))
			return Satisfiability.UNKNOWN;
		Satisfiability result = Satisfiability.BOTTOM;
		for (Type t : types)
			if (BoundMethods.isBoundMethod(t) || closureFunction(t) != null || t.isTypeTokenType()
					|| t.isCodeMemberType() || (isObject(t) && callable(t)))
				result = result.lub(Satisfiability.SATISFIED);
			else if ((PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
					|| (isObject(t) && PyBinaryDispatch.fullyKnown(t))
					|| ListModel.isList(t) || DictModel.isDict(t) || Unpacking.isTuple(t))
				result = result.lub(Satisfiability.NOT_SATISFIED);
			else
				return Satisfiability.UNKNOWN;
		return result;
	}

	/**
	 * Yields whether the callee might hold functions, natives, or closures.
	 *
	 * @return whether something is called
	 */
	boolean callsSomething() {
		return !functions.isEmpty() || !natives.isEmpty() || !closures.isEmpty() || !boundMethods.isEmpty()
				|| !callableObjects.isEmpty() || !classes.isEmpty();
	}

	/**
	 * Yields whether the callee might hold a value that cannot be called.
	 *
	 * @return whether the callee might not be callable
	 */
	boolean notCallable() {
		return notCallable;
	}

	/**
	 * Yields whether the callee might hold a value whose callability is
	 * unknown.
	 *
	 * @return whether the callee might be unknown
	 */
	boolean unknown() {
		return unknown;
	}

	/**
	 * Calls the functions, natives, closures, and bound methods held by the
	 * callee, calls {@code __call__} on the objects it holds, and creates
	 * objects of the classes it holds (as {@link PyNewObj}). Closures are
	 * passed as the first argument of their functions (see
	 * {@link Cells#CLOSURE}), objects as the receiver of {@code __call__}, and
	 * bound methods are called on their objects (see
	 * {@link BoundMethods#call}). The calls are sub-expressions of
	 * {@code caller} (see {@link CallSites#within}).
	 *
	 * @param <A>             the kind of {@link AbstractLattice} of the state
	 * @param <D>             the kind of {@link AbstractDomain} of the analysis
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the functions and the natives are
	 *                            called
	 * @param closureState    the state where the closures, the bound methods,
	 *                            and {@code __call__} are called, where the
	 *                            values of the callee (passed to them as their
	 *                            first argument) are still available
	 * @param caller          the statement making the call
	 * @param call            the call, with the arguments only
	 * @param callee          the expression computing the callee, whose state
	 *                            is stored in {@code expressions}
	 * @param args            the values of the arguments
	 * @param expressions     the states of the sub-expressions
	 * @param metas           where the meta variables of the calls are added
	 *
	 * @return the result of the calls (bottom if nothing is called)
	 *
	 * @throws SemanticException if the analysis fails
	 */
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> call(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			AnalysisState<A> closureState,
			Statement caller,
			UnresolvedCall call,
			Expression callee,
			ExpressionSet[] args,
			StatementStore<A> expressions,
			Collection<Identifier> metas)
			throws SemanticException {
		AnalysisState<A> result = state.bottomExecution();
		if (!functions.isEmpty())
			result = result.lub(call(CallSites.within(new CFGCall(call, functions), caller), interprocedural, state,
					args, expressions, metas));
		if (!natives.isEmpty())
			result = result.lub(call(CallSites.within(new NativeCall(call, natives), caller), interprocedural, state,
					args, expressions, metas));
		if (!closures.isEmpty()) {
			// the closure is the first argument
			UnresolvedCall withClosure = new UnresolvedCall(call.getCFG(), call.getLocation(), call.getCallType(),
					call.getQualifier(), call.getTargetName(), call.getOrder(),
					ArrayUtils.insert(0, call.getParameters(), callee));
			for (Map.Entry<String, Set<SymbolicExpression>> entry : closures.entrySet()) {
				ExpressionSet[] params = ArrayUtils.insert(0, args, new ExpressionSet(entry.getValue()));
				CFGCall closureCall = CallSites.within(
						new CFGCall(withClosure, Set.of(closureFunctions.get(entry.getKey()))), caller);
				result = result.lub(call(closureCall, interprocedural, closureState, params, expressions, metas));
			}
		}
		if (!callableObjects.isEmpty()) {
			// o(...) is o.__call__(...)
			PyMethodCall dunder = CallSites.within(new PyMethodCall(call.getCFG(), call.getLocation(), "__call__",
					call.getOrder(), ArrayUtils.insert(0, call.getParameters(), callee)), caller);
			ExpressionSet[] params = ArrayUtils.insert(0, args, new ExpressionSet(callableObjects));
			result = result.lub(call(dunder, interprocedural, closureState, params, expressions, metas));
		}
		for (SymbolicExpression bound : boundMethods)
			// the bound method is called on its object, standing for the
			// callee in the call
			result = result.lub(BoundMethods.call(interprocedural, closureState, bound, callee, call.getParameters(),
					args, caller, expressions, metas));
		if (!classes.isEmpty() && !(DerivedLocation.source(call.getLocation()) instanceof SourceCodeLocation))
			// the object cannot be created without a location in the code
			result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state,
					new PushAny(Untyped.INSTANCE, call.getLocation()), call));
		else if (!classes.isEmpty()) {
			// C(...) creates an object of C, initialized by __init__
			SourceCodeLocation location = (SourceCodeLocation) DerivedLocation.source(call.getLocation());
			for (PyClassType cls : classes.values()) {
				PyNewObj creation = new PyNewObj(call.getCFG(), location, "__init__", cls, call.getParameters());
				creation.setParentStatement(caller);
				result = result.lub(creation.forwardSemanticsAux(interprocedural, state, args, expressions));
				metas.addAll(creation.getMetaVariables());
			}
		}
		return result;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> call(
			Call call,
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions,
			Collection<Identifier> metas)
			throws SemanticException {
		AnalysisState<A> result = call.forwardSemanticsAux(interprocedural, state, params, expressions);
		metas.addAll(call.getMetaVariables());
		return result;
	}

	/**
	 * Yields the function of the closures of the given type, if it is the type
	 * of a reference to a closure.
	 *
	 * @param t the type
	 *
	 * @return the function, or {@code null}
	 */
	static CFG closureFunction(
			Type t) {
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType())
			return null;
		Unit unit = t.asPointerType().getInnerType().asUnitType().getUnit();
		return unit instanceof PyClosureUnit ? ((PyClosureUnit) unit).getFunction() : null;
	}

	// whether t is a class of the program (i.e., defined in its code)
	private static boolean isProgramClass(
			Type t) {
		return t instanceof PyClassType && ((PyClassType) t).getUnit() instanceof ClassUnit
				&& !(((PyClassType) t).getUnit() instanceof PyClosureUnit)
				&& DerivedLocation.source(((PyClassType) t).getUnit().getLocation()) instanceof SourceCodeLocation;
	}

	// whether t is the type of the objects of a class
	private static boolean isObject(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit() instanceof CompilationUnit;
	}

	// whether the class of the objects of type t defines __call__
	private static boolean callable(
			Type t) {
		CompilationUnit unit = (CompilationUnit) t.asPointerType().getInnerType().asUnitType().getUnit();
		return !unit.getInstanceCodeMembersByName("__call__", true).isEmpty();
	}
}
