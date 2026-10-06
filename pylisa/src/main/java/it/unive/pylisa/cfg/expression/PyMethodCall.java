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
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
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
 * are not complete.
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
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> receiverTypes = new HashSet<>();
		for (SymbolicExpression receiver : params[0])
			receiverTypes.addAll(analysis.getRuntimeTypesOf(state, receiver, this));

		Set<Type> valueTypes = new HashSet<>();
		boolean others = receiverTypes.isEmpty();
		for (Type t : receiverTypes)
			if (PyBinaryDispatch.isBuiltinValueType(t))
				valueTypes.add(t);
			else
				others = true;

		AnalysisState<A> result = state.bottom();
		if (others)
			// objects, or unknown receivers: regular call resolution
			result = result.lub(super.forwardSemanticsAux(interprocedural, state, params, expressions));

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
}
