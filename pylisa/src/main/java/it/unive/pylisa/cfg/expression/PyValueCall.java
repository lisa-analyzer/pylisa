package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.NativeCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
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

		Set<CFG> cfgs = new HashSet<>();
		Set<NativeCFG> natives = new HashSet<>();
		Set<SymbolicExpression> bounds = new HashSet<>();
		boolean unknown = false, notCallable = false;
		for (SymbolicExpression callee : params[0]) {
			Set<Type> types = analysis.getRuntimeTypesOf(state, callee, this);
			if (types.isEmpty() || PyBinaryDispatch.unknownTypes(types, this)) {
				unknown = true;
				continue;
			}
			for (Type t : types)
				if (BoundMethods.isBoundMethod(t))
					bounds.add(callee);
				else if (PyBinaryDispatch.isBuiltinValueType(t) && PyBinaryDispatch.fullyKnown(t))
					// e.g. an int, that cannot be called
					notCallable = true;
				else if (isObject(t) && PyBinaryDispatch.fullyKnown(t) && !callable(t))
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
					// e.g. a class, or an object whose class defines __call__
					unknown = true;
		}

		AnalysisState<A> result = state.bottomExecution();
		if (notCallable)
			result = result.lub(PyExceptions.raise(analysis, consumed, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR));
		if (!cfgs.isEmpty())
			result = result.lub(call(new CFGCall(call, cfgs), interprocedural, consumed, args, expressions));
		if (!natives.isEmpty())
			result = result.lub(call(new NativeCall(call, natives), interprocedural, consumed, args, expressions));
		if (unknown)
			result = result.lub(analysis.smallStepSemantics(consumed, new PushAny(Untyped.INSTANCE, getLocation()),
					this));
		for (SymbolicExpression bound : bounds)
			// the bound method is read before its value is consumed
			result = result.lub(BoundMethods.call(interprocedural, state, bound, subs[0],
					Arrays.copyOfRange(subs, 1, subs.length), args, this, expressions, getMetaVariables())
					.forgetIdentifiers(subs[0].getMetaVariables(), this));
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
