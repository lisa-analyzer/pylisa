package it.unive.pylisa.cfg.expression;

import org.apache.commons.lang3.ArrayUtils;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;

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
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural, AnalysisState<A> state, ExpressionSet[] params,
			StatementStore<A> expressions) throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Type type = getStaticType();
		ReferenceType reftype = new ReferenceType(type);
		MemoryAllocation created = new MemoryAllocation(type, getLocation(), false);
		HeapReference ref = new HeapReference(reftype, created, getLocation());

		// we need to add the receiver to the parameters
		VariableRef paramThis = new VariableRef(getCFG(), getLocation(), "$self", reftype);
		Expression[] fullExpressions = ArrayUtils.insert(0, getSubExpressions(), paramThis);

		// we also have to add the receiver inside the state
		AnalysisState<A> callstate = paramThis.forwardSemantics(state, interprocedural, expressions);
		AnalysisState<A> tmp = state.bottom();
		for (SymbolicExpression v : callstate.getExecutionExpressions())
			tmp = tmp.lub(analysis.assign(callstate, v, ref, paramThis));
		ExpressionSet[] fullParams = ArrayUtils.insert(0, params, callstate.getExecutionExpressions());
		expressions.put(paramThis, tmp);

		UnresolvedCall call = new UnresolvedCall(getCFG(), getLocation(), CallType.INSTANCE, type.toString(),
				getConstructName(), fullExpressions);
		AnalysisState<A> sem = call.forwardSemanticsAux(interprocedural, tmp, fullParams, expressions);

		if (!call.getMetaVariables().isEmpty())
			sem = sem.forgetIdentifiers(call.getMetaVariables(), this);

		// now remove the instrumented receiver
		expressions.forget(paramThis);
		for (SymbolicExpression v : callstate.getExecutionExpressions())
			if (v instanceof Identifier)
				sem = sem.forgetIdentifier((Identifier) v, this);

		sem = analysis.smallStepSemantics(sem, created, this);

		AnalysisState<A> result = state.bottom();
		for (SymbolicExpression loc : sem.getExecutionExpressions()) {
			ReferenceType staticType = new ReferenceType(loc.getStaticType());
			HeapReference locref = new HeapReference(staticType, loc, getLocation());
			result = result.lub(analysis.smallStepSemantics(sem, locref, call));
		}

		return result;
	}
}