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
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.HashSet;
import java.util.Set;

public class SequenceGetItem extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected SequenceGetItem(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression sequence,
			Expression index) {
		super(cfg, location, constructName, sequence, index);
	}

	public static SequenceGetItem build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SequenceGetItem(cfg, location, "__getitem__", exprs[0], exprs[1]);
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

	/**
	 * Native implementation of {@code Sequence.__getitem__(self, index)}:
	 * dereferences {@code self} as a heap pointer and reads the element at
	 * {@code index} as a heap child ({@link AccessChild}) &mdash; the same
	 * access path used to write each element when the sequence was built (e.g.
	 * by {@code ListCreation}), so indexing a list/tuple literal with a
	 * constant index resolves precisely. Whatever was written to that slot (a
	 * scalar, or itself a heap reference for a nested sequence) is returned
	 * as-is; no special-casing per element type is needed since the heap domain
	 * already tracks what was actually assigned there.
	 * <p>
	 * {@code Tuple} is immutable, so {@code TupleCreation} tracks the element
	 * count as a "length" heap field once and for all at creation time (unlike
	 * {@code List}, whose size can change via mutation methods, so no such
	 * field is tracked for it). For every runtime pointer type of {@code self}
	 * that resolves to (a subtype of) {@code Tuple}, this compares
	 * {@code index} against that field via {@code Analysis#satisfies} (backed
	 * by {@code ConstantPropagationDomain#satisfiesBinaryExpression}) and
	 * raises an {@code IndexError} (via {@link PyExceptions}) when the access
	 * is (possibly) out of range.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);

		AnalysisState<A> result = state.bottomExecution();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		boolean anyPointer = false;
		Set<Type> rts = analysis.getRuntimeTypesOf(state, left, context, this);
		for (Type t : rts) {
			if (!t.isPointerType())
				continue;
			anyPointer = true;
			Type inner = t.asPointerType().getInnerType();
			HeapDereference deref = new HeapDereference(inner, left, loc);
			// the executions where the sequence has this type (the type of a
			// reference can be told apart from the other ones, see
			// PyBinaryDispatch.checkable)
			AnalysisState<A> typed = PyBinaryDispatch.ofType(analysis, context, state, left, t, rts, this);
			if (typed.getExecutionState().isBottom())
				continue;

			if (inner.canBeAssignedTo(tupleType))
				result = result.lub(readTuple(interprocedural, typed, left, right, deref, raising));
			else if (ListModel.isList(t)) {
				// the index might be both a slice and something else (e.g. if
				// it is unknown)
				Set<Type> indexTypes = analysis.getRuntimeTypesOf(typed, right, context, this);
				if (indexTypes.stream().anyMatch(ListModel::isSlice))
					// lst[start:stop:step]
					result = result.lub(ListModel.slice(interprocedural, typed, left, right, this, st, raising));
				if (indexTypes.isEmpty() || !indexTypes.stream().allMatch(ListModel::isSlice))
					// bounds and index checked on the elements of the list
					result = result.lub(ListModel.read(analysis, context, typed, left, right, st, raising));
			} else if (DictModel.isDict(t))
				// the entries of the dictionary
				result = result.lub(DictModel.read(analysis, context, typed, left, right, st, raising));
			else {
				AccessChild access = new AccessChild(Untyped.INSTANCE, deref, right, loc);
				result = result.lub(analysis.smallStepSemantics(typed, access, context, st));
			}
		}
		if (!anyPointer) {
			HeapDereference deref = new HeapDereference(Untyped.INSTANCE, left, loc);
			AccessChild access = new AccessChild(Untyped.INSTANCE, deref, right, loc);
			result = result.lub(analysis.smallStepSemantics(state, access, context, st));
		}

		return raising.moveFrom(result);
	}

	// reads the element of a tuple at an int index, or a slice of it
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> readTuple(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression tuple,
			SymbolicExpression index,
			HeapDereference deref,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		AnalysisState<A> result = state.bottomExecution();
		Set<Type> types = analysis.getRuntimeTypesOf(state, index, context, this);
		boolean unknown = types.isEmpty() || types.stream().anyMatch(Type::isUntyped);
		boolean slices = unknown || types.stream().anyMatch(ListModel::isSlice);
		boolean ints = unknown || types.stream().anyMatch(SequenceGetItem::isIntegral);

		if (ints) {
			// the index is out of range surely only if it is surely an int
			boolean surely = !unknown && types.stream().allMatch(SequenceGetItem::isIntegral);
			result = result.lub(readWithBoundsCheck(analysis, context, state, deref, index, loc, surely, raising));
		}

		if (slices && ListModel.checkSlice(analysis, context, state, index, this, st, raising)) {
			// t[start:stop:step]: a new tuple, whose length and elements are
			// not known (allocated at a location different from the one of
			// the subscript, see DerivedLocation)
			DerivedLocation at = new DerivedLocation(loc, "slice");
			AnalysisState<A> created = TupleCreation.create(analysis, context, state, st, at, new ExpressionSet[0],
					getMetaVariables());
			for (SymbolicExpression copy : created.getExecutionExpressions())
				result = result.lub(TupleCreation.unknownLength(analysis, context, created, copy, st, at));
		}

		// the index of a tuple must be an int or a slice (when its type is not
		// known, the error is raised under an unknown condition)
		if (unknown)
			raising.raise(Satisfiability.UNKNOWN, state, getCFG(), loc, st, LibrarySpecificationProvider.TYPE_ERROR);
		else {
			Set<Type> failing = new HashSet<>();
			for (Type t : types)
				if (!isIntegral(t) && !ListModel.isSlice(t))
					failing.add(t);
			raising.raiseOfTypes(state, index, failing, types, true, getCFG(), loc, st,
					LibrarySpecificationProvider.TYPE_ERROR);
		}
		return result;
	}

	// whether the values of the type can index a sequence (ints and bools)
	private static boolean isIntegral(
			Type t) {
		return (t.isNumericType() && t.asNumericType().isIntegral()) || t.isBooleanType();
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> readWithBoundsCheck(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			HeapDereference deref,
			SymbolicExpression index,
			CodeLocation loc,
			boolean surely,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		GlobalVariable lenKey = new GlobalVariable(Int32Type.INSTANCE, "length", loc);
		AccessChild lenAccess = new AccessChild(Int32Type.INSTANCE, deref, lenKey, loc);
		// the length field must be resolved via smallStepSemantics (like any
		// other heap read) before it can be used as an operand of a further
		// symbolic expression evaluated by satisfies(...)
		AnalysisState<A> lenState = analysis.smallStepSemantics(state, lenAccess, context, this);

		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression lenId : lenState.getExecutionExpressions()) {
			// negative indexes count from the end: the index is out of range
			// if it is below -len or not below len
			it.unive.lisa.symbolic.value.BinaryExpression tooLow = new it.unive.lisa.symbolic.value.BinaryExpression(
					BoolType.INSTANCE, index,
					new UnaryExpression(Int32Type.INSTANCE, lenId, NumericNegation.INSTANCE, loc),
					ComparisonLt.INSTANCE, loc);
			it.unive.lisa.symbolic.value.BinaryExpression tooHigh = new it.unive.lisa.symbolic.value.BinaryExpression(
					BoolType.INSTANCE, index, lenId, ComparisonGe.INSTANCE, loc);
			it.unive.lisa.symbolic.value.BinaryExpression outOfRange = new it.unive.lisa.symbolic.value.BinaryExpression(
					BoolType.INSTANCE, tooLow, tooHigh, LogicalOr.INSTANCE, loc);

			// the index is out of range only if it is an int: otherwise, the
			// error is not surely an IndexError
			AnalysisState<A> reading = raising.raiseWhere(outOfRange, surely, lenState, this, getCFG(), loc, st,
					LibrarySpecificationProvider.INDEX_ERROR);
			if (!reading.getExecutionState().isBottom()) {
				AccessChild access = new AccessChild(Untyped.INSTANCE, deref, index, loc);
				result = result.lub(analysis.smallStepSemantics(reading, access, context, st));
			}
		}

		return result;
	}
}
