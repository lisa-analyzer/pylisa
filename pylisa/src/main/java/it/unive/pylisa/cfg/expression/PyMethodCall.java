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
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.PyMro;
import it.unive.pylisa.libraries.BuiltinAttributes;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
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
		Set<CFG> statics = new HashSet<>();
		// the receivers whose methods are resolved by the regular call
		// resolution
		Set<Type> objectTypes = new HashSet<>();
		boolean others = receiverTypes.isEmpty(), attribute = false;
		for (Type t : receiverTypes)
			if (!staticMethods(t).isEmpty())
				// o.m(...), where the class of o defines the static method m
				statics.addAll(staticMethods(t));
			else if (PyBinaryDispatch.isBuiltinValueType(t))
				valueTypes.add(t);
			else if (isAttribute(t))
				attribute = true;
			else {
				others = true;
				objectTypes.add(t);
			}

		AnalysisState<A> result = state.bottomExecution();
		if (others) {
			AnalysisState<A> resolving = state;
			if (receiverTypes.stream().anyMatch(ListModel::isList) && ListModel.unmodeled(getTargetName()))
				// a method of lists that is not modeled (e.g. pop) might
				// modify the list in any way
				for (SymbolicExpression receiver : params[0])
					resolving = ListModel.invalidate(analysis, resolving, receiver, this);
			if (receiverTypes.stream().anyMatch(DictModel::isDict) && DictModel.unmodeled(getTargetName()))
				// a method of dictionaries that is not modeled (e.g. pop) might
				// modify the dictionary in any way
				for (SymbolicExpression receiver : params[0])
					resolving = DictModel.invalidate(analysis, resolving, receiver, this);
			// methods of objects, or unknown receivers: regular call
			// resolution
			if (!objectTypes.isEmpty() && objectTypes.stream().allMatch(this::definedMethod)
					&& resolve(interprocedural, analysis, resolving, expressions) instanceof OpenCall)
				// the classes of the receivers, whose methods are all known,
				// define the method, but it does not accept the arguments (e.g.
				// a method without self)
				result = result.lub(PyExceptions.raise(analysis, resolving, getCFG(), getLocation(), this,
						LibrarySpecificationProvider.TYPE_ERROR));
			else
				result = result.lub(super.forwardSemanticsAux(interprocedural, resolving, params, expressions));
		}
		if (attribute)
			result = result.lub(callAttribute(interprocedural, state, params, expressions));
		if (!statics.isEmpty()) {
			// the static method is called without the receiver
			Expression[] parameters = getParameters();
			UnresolvedCall call = new UnresolvedCall(getCFG(), getLocation(), CallType.STATIC, null,
					getTargetName(), getOrder(), Arrays.copyOfRange(parameters, 1, parameters.length));
			result = result.lub(call(CallSites.within(new CFGCall(call, statics), this), interprocedural, state,
					Arrays.copyOfRange(params, 1, params.length), expressions));
		}

		if (valueTypes.isEmpty())
			return result;

		Set<Type>[] types = parameterTypes(expressions, analysis);
		boolean known = !PyBinaryDispatch.unknownTypes(receiverTypes, this);
		for (Type t : valueTypes) {
			types[0] = Set.of(t);
			Call resolved = PyBinaryDispatch.resolveInClass(interprocedural, state, this,
					PyBinaryDispatch.classOf(t), getTargetName(), getParameters(), types);
			if (resolved == null && known
					&& BuiltinAttributes.missing(PyBinaryDispatch.classOf(t), getTargetName()))
				// the values of the class have no such attribute (e.g.
				// (5).strip()), if the receiver is known to be of that class
				// (not just any value)
				result = result.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
						LibrarySpecificationProvider.ATTRIBUTE_ERROR));
			else if (resolved == null)
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

	// whether the objects of type t are of a class whose methods are all
	// known, defining an instance method named as the call
	private boolean definedMethod(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& PyBinaryDispatch.fullyKnown(t)
				&& !t.asPointerType().getInnerType().asUnitType().getUnit()
						.getInstanceCodeMembersByName(getTargetName(), true).isEmpty();
	}

	// the call resolved by the regular call resolution
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Call resolve(
			InterproceduralAnalysis<A, D> interprocedural,
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			StatementStore<A> expressions)
			throws SemanticException {
		try {
			return interprocedural.resolve(this, parameterTypes(expressions, analysis),
					state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve call " + this, e);
		}
	}

	// the static methods named as the call, defined by the class of the
	// objects of type t
	private Set<CFG> staticMethods(
			Type t) {
		Set<CFG> statics = new HashSet<>();
		if (t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType().getUnit() instanceof ClassUnit) {
			// the first class in the method resolution order defining it
			CompilationUnit definer = PyMro.definer(t.asPointerType().getInnerType().asUnitType().getUnit(),
					getTargetName(), null);
			if (definer != null)
				for (CodeMember member : definer.getCodeMembersByName(getTargetName()))
					if (member instanceof CFG && !member.getDescriptor().isInstance())
						statics.add((CFG) member);
		}
		return statics;
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

		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression receiver : params[0]) {
			Set<Type> receiverTypes = analysis.getRuntimeTypesOf(state, receiver, this);
			if (receiverTypes.stream().noneMatch(this::isAttribute))
				// handled by the regular call resolution
				continue;
			if (!receiverTypes.stream().allMatch(this::isAttribute)) {
				// the receiver might also be an object whose class defines the
				// method (handled by the regular call resolution): the
				// attribute cannot be read only from the other objects, and
				// its value is unknown, but reading it from them raises
				// AttributeError if they cannot have it
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, getLocation()),
						this));
				result = result.lub(read.missingErrors(interprocedural, state, receiver, this));
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
			Set<SymbolicExpression> bounds = new HashSet<>();
			boolean unknown = false, notCallable = false;
			for (SymbolicExpression value : attr.getExecutionExpressions()) {
				Set<Type> types = analysis.getRuntimeTypesOf(attr, value, this);
				if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this)) {
					unknown = true;
					continue;
				}
				for (Type t : types)
					if (BoundMethods.isBoundMethod(t))
						bounds.add(value);
					else if (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
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
				result = result.lub(call(CallSites.within(new CFGCall(call, cfgs), this), interprocedural, attr, args, expressions));
			if (!natives.isEmpty())
				result = result.lub(call(CallSites.within(new NativeCall(call, natives), this), interprocedural, attr, args, expressions));
			for (SymbolicExpression bound : bounds)
				// a bound method stored in the attribute, called on its object
				result = result.lub(BoundMethods.call(interprocedural, attr, bound, parameters[0],
						Arrays.copyOfRange(parameters, 1, parameters.length), args, this, expressions,
						getMetaVariables()));
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
