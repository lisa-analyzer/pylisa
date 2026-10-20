package it.unive.pylisa.libraries.lists;

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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;

/**
 * Native implementation of {@code list.copy(self)}, that returns a new list
 * with the same elements (a shallow copy). If the length of the list is known,
 * the elements are copied at the same indices; otherwise, all of them are
 * elements at unknown indices of the copy (see
 * {@link ListCreation#UNKNOWN_INDEX}), as the list might be any of many lists
 * with different lengths.
 */
public class ListCopy extends PyNative {

	protected ListCopy(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "copy", params);
	}

	public static ListCopy build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListCopy(cfg, location, exprs);
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
		// the copy is allocated at a location different from the one of the
		// call (see DerivedLocation)
		ListCreation creation = new ListCreation(getCFG(), new DerivedLocation(loc, "copy"));
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression self : params[0]) {
			HeapDereference source = new HeapDereference(listType, self, loc);
			ListModel.Shape shape = ListModel.shape(analysis, state, self, this);
			if (shape.length() != null) {
				// the elements at the same indices
				ExpressionSet[] elements = new ExpressionSet[shape.length()];
				for (int i = 0; i < elements.length; i++)
					elements[i] = analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, source,
							new Constant(Int32Type.INSTANCE, i, loc), loc), this).getExecutionExpressions();
				result = result.lub(creation.forwardSemanticsAux(interprocedural, state, elements, expressions));
				continue;
			}

			// all the elements, at unknown indices
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
					expressions);
			for (SymbolicExpression copy : created.getExecutionExpressions()) {
				AccessChild unknown = new AccessChild(Untyped.INSTANCE, new HeapDereference(listType, copy, loc),
						new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, loc), loc);
				AnalysisState<A> copied = created.bottomExecution();
				if (shape.elements() == null)
					copied = analysis.assign(created, unknown, new PushAny(Untyped.INSTANCE, loc), this);
				else
					for (SymbolicExpression field : shape.elements())
						for (SymbolicExpression element : analysis.smallStepSemantics(created,
								new AccessChild(Untyped.INSTANCE, source, field, loc), this).getExecutionExpressions())
							copied = copied.lub(analysis.assign(created, unknown, element, this));
				if (copied.getExecutionState().isBottom())
					// the list is empty: so is the copy
					copied = created;
				result = result.lub(copied.withExecutionExpression(copy));
			}
		}
		getMetaVariables().addAll(creation.getMetaVariables());
		return result;
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		// never invoked: forwardSemanticsAux computes the result
		throw new SemanticException("copy is evaluated by forwardSemanticsAux");
	}
}
