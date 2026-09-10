package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
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
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
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
	 * access path used to write each element when the sequence was built
	 * (e.g. by {@code ListCreation}), so indexing a list/tuple literal with a
	 * constant index resolves precisely. Whatever was written to that slot
	 * (a scalar, or itself a heap reference for a nested sequence) is
	 * returned as-is; no special-casing per element type is needed since the
	 * heap domain already tracks what was actually assigned there.
	 *
	 * <p>
	 * {@code Tuple} is immutable, so {@code TupleCreation} tracks the
	 * element count as a "length" heap field once and for all at creation
	 * time (unlike {@code List}, whose size can change via mutation methods,
	 * so no such field is tracked for it). For every runtime pointer type of
	 * {@code self} that resolves to (a subtype of) {@code Tuple}, this
	 * compares {@code index} against that field via {@code Analysis#satisfies}
	 * (backed by {@code ConstantPropagationDomain#satisfiesBinaryExpression})
	 * and raises an {@code IndexError} (via {@link PyExceptions}) when the
	 * access is (possibly) out of range.
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);

		AnalysisState<A> result = state.bottom();
		boolean anyPointer = false;
		Set<Type> rts = analysis.getRuntimeTypesOf(state, left, this);
		for (Type t : rts) {
			if (!t.isPointerType())
				continue;
			anyPointer = true;
			Type inner = t.asPointerType().getInnerType();
			HeapDereference deref = new HeapDereference(inner, left, loc);

			if (inner.canBeAssignedTo(tupleType))
				result = result.lub(readWithBoundsCheck(analysis, state, deref, right, loc));
			else {
				// TODO List has no bounds checking: lst[3] on a 3-element
				// list silently reads an untracked (TOP) slot instead of
				// raising IndexError, since (unlike Tuple) no length field
				// is tracked here (a cached length would go stale once
				// mutation methods like append/pop are implemented).
				AccessChild access = new AccessChild(Untyped.INSTANCE, deref, right, loc);
				result = result.lub(analysis.smallStepSemantics(state, access, st));
			}
		}

		if (!anyPointer) {
			HeapDereference deref = new HeapDereference(Untyped.INSTANCE, left, loc);
			AccessChild access = new AccessChild(Untyped.INSTANCE, deref, right, loc);
			result = result.lub(analysis.smallStepSemantics(state, access, st));
		}

		return result;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> readWithBoundsCheck(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			HeapDereference deref,
			SymbolicExpression index,
			CodeLocation loc)
			throws SemanticException {
		Variable lenKey = new Variable(Int32Type.INSTANCE, "length", loc);
		AccessChild lenAccess = new AccessChild(Int32Type.INSTANCE, deref, lenKey, loc);
		// the length field must be resolved via smallStepSemantics (like any
		// other heap read) before it can be used as an operand of a further
		// symbolic expression evaluated by satisfies(...)
		AnalysisState<A> lenState = analysis.smallStepSemantics(state, lenAccess, this);

		AnalysisState<A> result = state.bottom();
		for (SymbolicExpression lenId : lenState.getExecutionExpressions()) {
			it.unive.lisa.symbolic.value.BinaryExpression tooLow = new it.unive.lisa.symbolic.value.BinaryExpression(
					BoolType.INSTANCE, index, new Constant(Int32Type.INSTANCE, 0, loc), ComparisonLt.INSTANCE, loc);
			it.unive.lisa.symbolic.value.BinaryExpression tooHigh = new it.unive.lisa.symbolic.value.BinaryExpression(
					BoolType.INSTANCE, index, lenId, ComparisonGe.INSTANCE, loc);

			Satisfiability sat = analysis.satisfies(lenState, tooLow, this)
					.or(analysis.satisfies(lenState, tooHigh, this));

			if (sat != Satisfiability.SATISFIED) {
				AccessChild access = new AccessChild(Untyped.INSTANCE, deref, index, loc);
				result = result.lub(analysis.smallStepSemantics(lenState, access, st));
			}
			if (sat != Satisfiability.NOT_SATISFIED)
				result = result.lub(PyExceptions.raise(analysis, lenState, getCFG(), loc, this,
						LibrarySpecificationProvider.INDEX_ERROR));
		}

		return result;
	}
}
