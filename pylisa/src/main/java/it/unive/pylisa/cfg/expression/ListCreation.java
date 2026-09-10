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
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class ListCreation extends NaryExpression {

	public ListCreation(
			CFG cfg,
			CodeLocation loc,
			Expression... values) {
		super(cfg, loc, "list", values);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();

		Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
		ReferenceType reftype = new ReferenceType(listType);
		MemoryAllocation created = new MemoryAllocation(listType, loc, false);
		HeapReference ref = new HeapReference(reftype, created, loc);
		HeapDereference deref = new HeapDereference(listType, ref, loc);

		AnalysisState<A> current = analysis.smallStepSemantics(state, ref, this);

		for (int i = 0; i < params.length; i++) {
			AccessChild slot = new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, i, loc), loc);
			AnalysisState<A> slotState = analysis.smallStepSemantics(current, slot, this);

			AnalysisState<A> next = state.bottom();
			for (SymbolicExpression element : params[i])
				for (SymbolicExpression slotId : slotState.getExecutionExpressions())
					next = next.lub(analysis.assign(slotState, slotId, element, this));
			current = next;
		}

		return analysis.smallStepSemantics(current, ref, this);
	}
}
