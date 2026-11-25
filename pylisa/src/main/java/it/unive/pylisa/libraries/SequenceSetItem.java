package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collections;
import java.util.Set;

/**
 * Native implementation of {@code Sequence.__setitem__(self, index, value)}:
 * dereferences {@code self} as a heap pointer and writes {@code value} into the
 * child identified by {@code index} ({@link AccessChild}) &mdash; the same
 * access path {@code SequenceGetItem} reads from and {@code ListCreation}
 * writes to when the sequence is built, so {@code lst[i] = v} followed by
 * {@code lst[i]} resolves precisely for a constant index.
 * <p>
 * {@code Tuple} inherits this method from {@code Sequence} (real Python tuples
 * do not define their own {@code __setitem__}), but tuples are immutable:
 * assigning to {@code t[i]} raises a {@code TypeError}. For every runtime
 * pointer type of {@code self} that resolves to (a subtype of) {@code Tuple},
 * this raises that error (via {@link PyExceptions}) instead of performing the
 * write.
 */
public class SequenceSetItem extends NaryExpression implements PluggableStatement {

	protected Statement st;

	protected SequenceSetItem(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression[] params) {
		super(cfg, location, constructName, params);
	}

	public static SequenceSetItem build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SequenceSetItem(cfg, location, "__setitem__", exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
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
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);

		AnalysisState<A> result = state.bottomExecution();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		for (SymbolicExpression self : params[0]) {
			// each combination of the operands is evaluated in the executions
			// computing it, as the statements of lisa do: if they are not
			// known exactly, the exceptions are raised under an unknown
			// condition
			AnalysisState<A> selfState = restrictTo(interprocedural, state, self);
			if (selfState.getExecutionState().isBottom())
				continue;
			boolean selfExact = params[0].size() == 1 || restrictsExactly(interprocedural, state, self);
			PyExceptions.Raising<A, D> selfRaising = raising.certainOnlyIf(selfExact);
			Set<Type> rts = analysis.getRuntimeTypesOf(selfState, self, context, this);
			for (Type t : rts) {
				if (!t.isPointerType())
					continue;
				Type inner = t.asPointerType().getInnerType();

				if (inner.canBeAssignedTo(tupleType)) {
					// tuples are immutable: raised where self is this tuple
					selfRaising.raiseOfTypes(selfState, self, Collections.singleton(t), rts, true, getCFG(), loc, st,
							LibrarySpecificationProvider.TYPE_ERROR);
					continue;
				}

				// the executions where self has this type (the type of a
				// reference can be told apart from the other ones, see
				// PyBinaryDispatch.checkable)
				AnalysisState<A> typed = PyBinaryDispatch.ofType(analysis, context, selfState, self, t, rts, this);
				if (typed.getExecutionState().isBottom())
					continue;

				for (SymbolicExpression index : params[1]) {
					AnalysisState<A> indexState = restrictTo(interprocedural, typed, null, index);
					boolean indexExact = params[1].size() == 1
							|| restrictsExactly(interprocedural, typed, null, index);
					for (SymbolicExpression value : params[2]) {
						AnalysisState<A> combination = restrictTo(interprocedural, indexState, null, null, value);
						if (combination.getExecutionState().isBottom())
							continue;
						PyExceptions.Raising<A, D> combinationRaising = raising.certainOnlyIf(selfExact && indexExact
								&& (params[2].size() == 1
										|| restrictsExactly(interprocedural, indexState, null, null, value)));
						if (ListModel.isList(t)) {
							// the elements of the list, or its slices
							Set<Type> indexTypes = analysis.getRuntimeTypesOf(combination, index, context, this);
							boolean slice = !indexTypes.isEmpty() && indexTypes.stream().allMatch(ListModel::isSlice);
							result = result.lub(slice
									? ListModel.assignSlice(analysis, context, combination, self, index, value, st,
											getMetaVariables(), combinationRaising)
									: ListModel.write(analysis, context, combination, self, index, value, st,
											combinationRaising));
						} else if (DictModel.isDict(t))
							// the entries of the dictionary
							result = result.lub(
									DictModel.write(analysis, context, combination, self, index, value, st,
											combinationRaising));
						else {
							HeapDereference deref = new HeapDereference(inner, self, loc);
							AccessChild slot = new AccessChild(Untyped.INSTANCE, deref, index, loc);
							AnalysisState<A> slotState = analysis.smallStepSemantics(combination, slot, context, this);
							for (SymbolicExpression slotId : slotState.getExecutionExpressions())
								result = result.lub(analysis.assign(slotState, slotId, value, context, this));
						}
					}
				}
			}
		}

		return raising.moveFrom(result);
	}
}
