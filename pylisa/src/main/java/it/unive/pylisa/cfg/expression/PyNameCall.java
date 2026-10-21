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
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
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
	public PyNameCall(
			CFG cfg,
			CodeLocation location,
			String name,
			EvaluationOrder order,
			Expression... parameters) {
		super(cfg, location, CallType.STATIC, null, name, order, parameters);
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
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);

		Set<CFG> cfgs = new HashSet<>();
		Set<NativeCFG> natives = new HashSet<>();
		boolean others = false, notCallable = false, bound = false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, callee, this);
		if (PyBinaryDispatch.unknownTypes(types, this))
			// e.g. a library function (as in f = len), whose type is unknown
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);
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
			return super.forwardSemanticsAux(interprocedural, state, params, expressions);

		AnalysisState<A> result = state.bottomExecution();
		if (notCallable)
			result = result.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
					LibrarySpecificationProvider.TYPE_ERROR));
		if (!cfgs.isEmpty())
			result = result.lub(call(new CFGCall(this, cfgs), interprocedural, state, params, expressions));
		if (!natives.isEmpty())
			result = result.lub(call(new NativeCall(this, natives), interprocedural, state, params, expressions));
		if (bound)
			// a bound method, called on its object (this call stands for it)
			result = result.lub(BoundMethods.call(interprocedural, state, callee, this, getParameters(), params, this,
					expressions, getMetaVariables()));
		if (others)
			// the variable might also hold something else
			result = result.lub(super.forwardSemanticsAux(interprocedural, state, params, expressions));
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
