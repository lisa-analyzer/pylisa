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
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.Set;

/**
 * Native implementation of {@code Sequence.__contains__(self, item)} (element
 * membership for {@code list}/{@code set}/{@code dict}/{@code tuple}). There
 * is no element-tracking abstract domain for sequences in this codebase, so
 * this is imprecise in general (always {@code top}).
 *
 * <p>
 * {@code Tuple} is the exception: like {@link SequenceLen} and
 * {@link SequenceGetItem}, it leverages the "length" heap field
 * {@code TupleCreation} tracks at creation time to walk every slot
 * {@code 0..length-1} (bounded via {@code Analysis#satisfies}, mirroring
 * {@code SequenceGetItem}'s bounds check) and compares {@code item} against
 * each one with {@code ComparisonEq}, OR-ing the per-slot verdicts together
 * ({@link Satisfiability#or}): {@code SATISFIED} if any slot definitely
 * equals {@code item}, {@code NOT_SATISFIED} if none of them possibly do
 * (including the empty-tuple case), {@code UNKNOWN} otherwise.
 */
public class SequenceContains extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected SequenceContains(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression item) {
		super(cfg, location, constructName, self, item);
	}

	public static SequenceContains build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SequenceContains(cfg, location, "__contains__", exprs[0], exprs[1]);
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

			if (inner.canBeAssignedTo(tupleType))
				// since for tuple we track the length and the slots, we can be precise
				result = result.lub(containsPrecise(analysis, state, inner, left, right, loc));
			else
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(BoolType.INSTANCE, loc), st));
		}

		if (!anyPointer)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(BoolType.INSTANCE, loc), st));

		return result;
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> containsPrecise(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Type innerType,
			SymbolicExpression self,
			SymbolicExpression item,
			CodeLocation loc)
			throws SemanticException {
		HeapDereference deref = new HeapDereference(innerType, self, loc);
		Variable lenKey = new Variable(Int32Type.INSTANCE, "length", loc);
		AccessChild lenAccess = new AccessChild(Int32Type.INSTANCE, deref, lenKey, loc);
		AnalysisState<A> lenState = analysis.smallStepSemantics(state, lenAccess, this);

		AnalysisState<A> result = state.bottom();
		for (SymbolicExpression lenId : lenState.getExecutionExpressions()) {
			Satisfiability overall = Satisfiability.NOT_SATISFIED;
			AnalysisState<A> loopState = lenState;

			for (int i = 0;; i++) {
				Constant idx = new Constant(Int32Type.INSTANCE, i, loc);
				it.unive.lisa.symbolic.value.BinaryExpression inBounds = new it.unive.lisa.symbolic.value.BinaryExpression(
						BoolType.INSTANCE, idx, lenId, ComparisonLt.INSTANCE, loc);
				if (analysis.satisfies(loopState, inBounds, this) != Satisfiability.SATISFIED)
					break;

				AccessChild slot = new AccessChild(Untyped.INSTANCE, deref, idx, loc);
				AnalysisState<A> slotState = analysis.smallStepSemantics(loopState, slot, this);
				for (SymbolicExpression slotId : slotState.getExecutionExpressions()) {
					it.unive.lisa.symbolic.value.BinaryExpression eq = new it.unive.lisa.symbolic.value.BinaryExpression(
							BoolType.INSTANCE, item, slotId, ComparisonEq.INSTANCE, loc);
					overall = overall.or(analysis.satisfies(slotState, eq, this));
					loopState = slotState;
				}
			}

			Constant verdict;
			if (overall == Satisfiability.SATISFIED)
				verdict = new Constant(BoolType.INSTANCE, true, loc);
			else if (overall == Satisfiability.NOT_SATISFIED)
				verdict = new Constant(BoolType.INSTANCE, false, loc);
			else {
				result = result.lub(analysis.smallStepSemantics(loopState, new PushAny(BoolType.INSTANCE, loc), st));
				continue;
			}
			result = result.lub(analysis.smallStepSemantics(loopState, verdict, st));
		}

		return result;
	}
}
