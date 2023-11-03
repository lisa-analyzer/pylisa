package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import java.util.Collections;
import java.util.HashSet;
import org.apache.commons.lang3.ArrayUtils;

public class PyNewObj extends NaryExpression {

	/**
	 * Builds the object allocation and initialization.
	 *
	 * @param cfg        the {@link CFG} where this operation lies
	 * @param type       the type of the object that is being created
	 * @param parameters the parameters of the constructor call
	 */
	public PyNewObj(
			CFG cfg,
			SourceCodeLocation location,
			String constructName,
			Type type,
			Expression... parameters) {
		super(cfg, location, constructName, type, parameters);
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> expressionSemantics(
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					AnalysisState<A, H, V, T> state,
					ExpressionSet<SymbolicExpression>[] params,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		Type type = getStaticType();
		ReferenceType reftype = new ReferenceType(type);
		MemoryAllocation created = new MemoryAllocation(type, getLocation(), false);
		HeapReference ref = new HeapReference(reftype, created, getLocation());
		created.setRuntimeTypes(Collections.singleton(type));
		ref.setRuntimeTypes(Collections.singleton(reftype));

		// we need to add the receiver to the parameters
		VariableRef paramThis = new VariableRef(getCFG(), getLocation(), "$self", reftype);
		Expression[] fullExpressions = ArrayUtils.insert(0, getSubExpressions(), paramThis);

		// we also have to add the receiver inside the state
		AnalysisState<A, H, V, T> callstate = paramThis.semantics(state, interprocedural, expressions);
		AnalysisState<A, H, V, T> tmp = state.bottom();
		HashSet<SymbolicExpression> expr = new HashSet<>();
		for (SymbolicExpression v : callstate.getComputedExpressions()) {
			tmp = tmp.lub(callstate.assign(v, ref, paramThis));
			expr.add(v);
		}
		ExpressionSet<SymbolicExpression>[] fullParams = ArrayUtils.insert(0, params, new ExpressionSet<>(expr));
		expressions.put(paramThis, tmp);

		UnresolvedCall call = new UnresolvedCall(getCFG(), getLocation(), CallType.INSTANCE, type.toString(),
				getConstructName(), fullExpressions);
		AnalysisState<A, H, V, T> sem = call.expressionSemantics(interprocedural, tmp, fullParams, expressions);

		if (!call.getMetaVariables().isEmpty())
			sem = sem.forgetIdentifiers(call.getMetaVariables());

		// now remove the instrumented receiver
		expressions.forget(paramThis);
		for (SymbolicExpression v : callstate.getComputedExpressions())
			if (v instanceof Identifier)
				sem = sem.forgetIdentifier((Identifier) v);

		sem = sem.smallStepSemantics(created, this);

		AnalysisState<A, H, V, T> result = state.bottom();
		for (SymbolicExpression loc : sem.getComputedExpressions()) {
			ReferenceType staticType = new ReferenceType(loc.getStaticType());
			HeapReference locref = new HeapReference(staticType, loc, getLocation());
			result = result.lub(sem.smallStepSemantics(locref, call));
		}

		return result;
	}
}