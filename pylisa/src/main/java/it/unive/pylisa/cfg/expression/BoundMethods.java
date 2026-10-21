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
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.CodeMemberType;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.PyMro;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/**
 * The bound methods of python: reading the method {@code m} of an object
 * {@code o} without calling it (e.g. {@code f = o.m}) yields an object of the
 * class {@code BoundMethod}, whose children are {@value #SELF} (that is,
 * {@code o}) and {@value #FUNC} (the function {@code m} of the class of
 * {@code o}). Calling it calls {@code m} on {@code o}: the method is looked up
 * by name in the class of each object {@code o} might be, as for
 * {@code o.m(...)} (see {@code PyMro}).
 */
public final class BoundMethods {

	/**
	 * The child holding the object the method is bound to.
	 */
	public static final String SELF = "__self__";

	/**
	 * The child holding the function of the method.
	 */
	public static final String FUNC = "__func__";

	private BoundMethods() {
	}

	/**
	 * Whether the values of the given type are bound methods.
	 *
	 * @param t the type
	 *
	 * @return whether the values are bound methods
	 */
	public static boolean isBoundMethod(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit().getName()
						.equals(LibrarySpecificationProvider.BOUND_METHOD);
	}

	/**
	 * Creates a bound method of the given object.
	 *
	 * @param analysis  the analysis
	 * @param state     the state where the object has been evaluated
	 * @param self      the object
	 * @param functions the functions the method might be
	 * @param pp        the statement reading the method
	 * @param metas     where the receiver of the bound method is added, as it
	 *                      is a meta variable that the statement using it must
	 *                      forget
	 *
	 * @return the state where the bound method has been created
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression self,
			Set<CodeMember> functions,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = new DerivedLocation(pp.getLocation(), "bound");
		Type type = PyClassType.lookup(LibrarySpecificationProvider.BOUND_METHOD);
		ReferenceType reftype = new ReferenceType(type);
		InstrumentedReceiver bound = new InstrumentedReceiver(reftype, false, loc);
		AnalysisState<A> current = analysis.assign(state, bound,
				new HeapReference(reftype, new MemoryAllocation(type, loc, false), loc), pp);
		HeapDereference deref = new HeapDereference(type, bound, loc);
		current = analysis.assign(current, child(deref, SELF, loc), self, pp);
		AnalysisState<A> withFunc = current.bottomExecution();
		for (CodeMember function : functions)
			withFunc = withFunc.lub(analysis.assign(current, child(deref, FUNC, loc),
					new Constant(new CodeMemberType(Set.of(function)), function, loc), pp));
		metas.add(bound);
		return analysis.smallStepSemantics(withFunc, bound, pp);
	}

	/**
	 * Calls the bound method with the given arguments: the method named as its
	 * function is called on its object.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the call happens
	 * @param bound           the bound method
	 * @param receiver        the expression standing for the object in the
	 *                            call (e.g. the callee)
	 * @param args            the arguments
	 * @param values          the values of the arguments
	 * @param caller          the statement calling the bound method
	 * @param expressions     the states of the sub-expressions of the caller
	 * @param metas           where the meta variables of the call are added
	 *
	 * @return the state after the call
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> call(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression bound,
			Expression receiver,
			Expression[] args,
			ExpressionSet[] values,
			Statement caller,
			StatementStore<A> expressions,
			Collection<Identifier> metas)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = caller.getLocation();
		HeapDereference deref = new HeapDereference(PyClassType.lookup(LibrarySpecificationProvider.BOUND_METHOD),
				bound, loc);
		AnalysisState<A> self = analysis.smallStepSemantics(state, child(deref, SELF, loc), caller);
		AnalysisState<A> func = analysis.smallStepSemantics(self, child(deref, FUNC, loc), caller);
		Set<String> names = new HashSet<>();
		boolean unknown = false;
		for (SymbolicExpression f : func.getExecutionExpressions())
			for (Type t : analysis.getRuntimeTypesOf(func, f, caller))
				if (t.isCodeMemberType())
					for (CodeMember member : t.asCodeMemberType().getMembers())
						names.add(member.getDescriptor().getName());
				else
					unknown = true;

		Expression[] params = new Expression[args.length + 1];
		params[0] = receiver;
		System.arraycopy(args, 0, params, 1, args.length);
		ExpressionSet[] actuals = new ExpressionSet[values.length + 1];
		actuals[0] = self.getExecutionExpressions();
		System.arraycopy(values, 0, actuals, 1, values.length);
		@SuppressWarnings("unchecked")
		Set<Type>[] types = new Set[actuals.length];
		for (int i = 0; i < actuals.length; i++) {
			types[i] = new HashSet<>();
			for (SymbolicExpression v : actuals[i])
				types[i].addAll(analysis.getRuntimeTypesOf(self, v, caller));
		}
		boolean fullyKnown = !types[0].isEmpty() && types[0].stream().allMatch(PyBinaryDispatch::fullyKnown);

		AnalysisState<A> result = state.bottomExecution();
		for (String name : names) {
			// a location of its own, as the callee might be the result of
			// another call at the location of the caller (as in o.f()())
			UnresolvedCall call = CallSites.get(interprocedural, caller, new DerivedLocation(loc, "bound call"),
					CallType.INSTANCE, null, name, params);
			Call resolved;
			try {
				resolved = interprocedural.resolve(call, types,
						self.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
			} catch (CallResolutionException e) {
				throw new SemanticException("Unable to resolve call " + call, e);
			}
			if (resolved instanceof OpenCall && fullyKnown)
				// the method does not accept the arguments
				result = result.lub(PyExceptions.raise(analysis, self, caller.getCFG(), loc, caller,
						LibrarySpecificationProvider.TYPE_ERROR));
			else {
				result = result.lub(resolved.forwardSemanticsAux(interprocedural, self, actuals, expressions));
				metas.addAll(resolved.getMetaVariables());
			}
		}
		if (unknown)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), caller));
		return result;
	}

	private static AccessChild child(
			HeapDereference deref,
			String name,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, name, loc), loc);
	}

	/**
	 * Yields the functions of the method {@code name} of the objects of type
	 * {@code t}, if their class defines it as a method that is not static (a
	 * static method read from an object is a plain function).
	 *
	 * @param t    the type of the objects
	 * @param name the name of the method
	 *
	 * @return the functions, empty if {@code name} is not such a method
	 */
	public static Set<CodeMember> methods(
			Type t,
			String name) {
		Set<CodeMember> methods = new HashSet<>();
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType())
			return methods;
		CompilationUnit definer = PyMro.definer(t.asPointerType().getInnerType().asUnitType().getUnit(), name,
				null);
		if (definer != null)
			for (CodeMember member : definer.getInstanceCodeMembersByName(name, false))
				if (member instanceof CFG)
					methods.add(member);
		return methods;
	}
}
