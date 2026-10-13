package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * A method call {@code receiver.name(args)}, where {@code receiver} is passed
 * as first parameter. Call resolution only finds methods of objects (i.e.,
 * instances of classes in memory): when the receiver is an instance of a
 * builtin class modeled as a value (e.g. {@code str} or {@code int}), the
 * method is instead looked up in the library class modeling it (e.g.
 * {@code "abc".upper()} calls {@code Str.upper("abc")}). A method that the
 * library class does not define yields an unknown value, as the library models
 * are not complete. When the receiver is an object whose class does not define
 * {@code name} (as a method or as a class attribute), {@code name} is an
 * attribute of the object (see {@link PyAttributeAccess}), whose value is
 * called: a function stored in the attribute is called without the receiver,
 * while calling a value that cannot be called (e.g. an {@code int}) raises
 * {@code TypeError}, and an attribute that has never been assigned raises
 * {@code AttributeError}.
 */
public class PyMethodCall extends UnresolvedCall {

	/**
	 * Builds the method call.
	 *
	 * @param cfg        the cfg where the call happens
	 * @param location   the location of the call
	 * @param name       the name of the method
	 * @param order      the evaluation order of the parameters
	 * @param parameters the parameters of the call, receiver first
	 */
	public PyMethodCall(
			CFG cfg,
			CodeLocation location,
			String name,
			EvaluationOrder order,
			Expression... parameters) {
		super(cfg, location, CallType.UNKNOWN, null, name, order, parameters);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		if (state.getExecutionState().isBottom())
			// unreachable: nothing is computed, and nothing is left on the
			// stack
			return state.withExecution(state.getExecution().bottom());
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> receiverTypes = new HashSet<>();
		for (SymbolicExpression receiver : params[0])
			receiverTypes.addAll(analysis.getRuntimeTypesOf(state, receiver, this));

		Set<Type> valueTypes = new HashSet<>();
		boolean others = receiverTypes.isEmpty(), attribute = false;
		for (Type t : receiverTypes)
			if (PyBinaryDispatch.isBuiltinValueType(t))
				valueTypes.add(t);
			else if (isAttribute(t))
				attribute = true;
			else
				others = true;

		AnalysisState<A> result = state.bottom();
		if (others)
			// methods of objects, or unknown receivers: regular call
			// resolution
			result = result.lub(super.forwardSemanticsAux(interprocedural, state, params, expressions));
		if (attribute)
			result = result.lub(callAttribute(interprocedural, state, params, expressions));

		if (valueTypes.isEmpty())
			return result;

		Set<Type>[] types = parameterTypes(expressions, analysis);
		for (Type t : valueTypes) {
			types[0] = Set.of(t);
			Call resolved = PyBinaryDispatch.resolveInClass(interprocedural, state, this,
					PyBinaryDispatch.classOf(t), getTargetName(), getParameters(), types);
			if (resolved == null)
				// the method is not modeled
				result = result.lub(analysis.smallStepSemantics(state,
						new PushAny(Untyped.INSTANCE, getLocation()), this));
			else {
				result = result.lub(resolved.forwardSemanticsAux(interprocedural, state, params, expressions));
				getMetaVariables().addAll(resolved.getMetaVariables());
			}
		}
		return result;
	}

	// whether name is an attribute of the objects of type t (or a class
	// attribute), rather than a method of their class
	private boolean isAttribute(
			Type t) {
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType()
				|| !PyBinaryDispatch.fullyKnown(t))
			return false;
		CompilationUnit unit = t.asPointerType().getInnerType().asUnitType().getUnit();
		return unit instanceof ClassUnit && !PyClassAttributes.isMethod((ClassUnit) unit, getTargetName());
	}

	// whether the objects of type t, whose class is fully known, can be
	// called
	private static boolean callable(
			Type t) {
		CompilationUnit unit = t.asPointerType().getInnerType().asUnitType().getUnit();
		return !unit.getInstanceCodeMembersByName("__call__", true).isEmpty();
	}

	// calls the value of the attribute name of the receiver
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> callAttribute(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Expression[] parameters = getParameters();
		PyAttributeAccess read = new PyAttributeAccess(getCFG(), getLocation(), parameters[0], getTargetName());
		// the value of the attribute is called without the receiver
		UnresolvedCall call = new UnresolvedCall(getCFG(), getLocation(), CallType.STATIC, null, getTargetName(),
				getOrder(), Arrays.copyOfRange(parameters, 1, parameters.length));
		ExpressionSet[] args = Arrays.copyOfRange(params, 1, params.length);

		AnalysisState<A> result = state.bottom();
		for (SymbolicExpression receiver : params[0]) {
			Set<Type> receiverTypes = analysis.getRuntimeTypesOf(state, receiver, this);
			if (receiverTypes.stream().noneMatch(this::isAttribute))
				// handled by the regular call resolution
				continue;
			if (!receiverTypes.stream().allMatch(this::isAttribute)) {
				// the receiver might also be an object whose class defines the
				// method (handled by the regular call resolution): the
				// attribute cannot be read only from the other objects, and
				// its value is unknown
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, getLocation()),
						this));
				continue;
			}

			// also raises AttributeError if the attribute is not assigned
			AnalysisState<A> attr = read.read(interprocedural, state, receiver, this);
			getMetaVariables().addAll(read.getMetaVariables());
			if (attr.getExecutionState().isBottom()) {
				result = result.lub(attr);
				continue;
			}

			Set<CFG> cfgs = new HashSet<>();
			Set<NativeCFG> natives = new HashSet<>();
			boolean unknown = false, notCallable = false;
			for (SymbolicExpression value : attr.getExecutionExpressions()) {
				Set<Type> types = analysis.getRuntimeTypesOf(attr, value, this);
				if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this)) {
					unknown = true;
					continue;
				}
				for (Type t : types)
					if (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
						// e.g. an int, that cannot be called
						notCallable = true;
					else if (isAttribute(t) && !callable(t))
						// an object whose class does not define __call__
						notCallable = true;
					else if (t.isCodeMemberType())
						for (CodeMember member : t.asCodeMemberType().getMembers())
							if (member instanceof CFG)
								cfgs.add((CFG) member);
							else if (member instanceof NativeCFG)
								natives.add((NativeCFG) member);
							else
								unknown = true;
					else
						unknown = true;
			}

			if (notCallable)
				result = result.lub(PyExceptions.raise(analysis, attr, getCFG(), getLocation(), this,
						LibrarySpecificationProvider.TYPE_ERROR));
			if (!cfgs.isEmpty())
				result = result.lub(call(new CFGCall(call, cfgs), interprocedural, attr, args, expressions));
			if (!natives.isEmpty())
				result = result.lub(call(new NativeCall(call, natives), interprocedural, attr, args, expressions));
			if (unknown)
				// e.g. a class, or an object
				result = result.lub(analysis.smallStepSemantics(attr, new PushAny(Untyped.INSTANCE, getLocation()),
						this));
		}
		return result;
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
}
