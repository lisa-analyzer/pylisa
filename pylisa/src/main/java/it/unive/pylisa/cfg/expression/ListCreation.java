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
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

/**
 * A list literal {@code [e0, ..., en]}: a new object is allocated, and the
 * receiver pointing to it (an {@link InstrumentedReceiver}) is the value of the
 * literal. The element at index {@code i} is stored in the child {@code i} (an
 * integer constant) of the object.
 */
public class ListCreation extends NaryExpression {

	public ListCreation(
			CFG cfg,
			CodeLocation loc,
			Expression... values) {
		super(cfg, loc, "list", values);
	}

	/**
	 * The child of a list holding its elements at unknown indices. It is not a
	 * valid python identifier, so that it cannot be an attribute of the list.
	 */
	public static final String UNKNOWN_INDEX = "$unknown";

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

		InstrumentedReceiver list = new InstrumentedReceiver(reftype, true, loc);
		AnalysisState<A> current = analysis.assign(state, list, ref, this);
		HeapDereference deref = new HeapDereference(listType, list, loc);
		// the heap tracks the fields accessed on each object, but until the
		// first access to any field in the program, the fields are unknown
		// for all objects (see PyNewObj): accessing __class__, that all
		// objects have, makes the elements of the new list known also when it
		// is empty
		AccessChild cls = new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, "__class__", loc),
				loc);
		current = analysis.smallStepSemantics(current, cls, this).withExecutionExpressions(
				current.getExecutionExpressions());

		for (int i = 0; i < params.length; i++) {
			AccessChild slot = new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, i, loc), loc);
			AnalysisState<A> next = state.bottom();
			for (SymbolicExpression element : params[i])
				next = next.lub(analysis.assign(current, slot, element, this));
			current = next;
		}

		// the values of the elements are consumed here
		for (Expression element : getSubExpressions())
			current = current.forgetIdentifiers(element.getMetaVariables(), this);
		getMetaVariables().add(list);
		return analysis.smallStepSemantics(current, list, this);
	}
}
