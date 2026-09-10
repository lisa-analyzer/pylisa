package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.Set;

/**
 * Native implementation of {@code Sequence.__len__(self)}. {@code Tuple}
 * tracks its element count as a "length" heap field at creation time (see
 * {@code TupleCreation} and {@link SequenceGetItem}), so for every runtime
 * pointer type of {@code self} that resolves to (a subtype of) {@code Tuple},
 * this reads that field directly instead of returning {@code top}. No such
 * field is tracked for {@code List} (its size can change via mutation
 * methods) nor for {@code Set}/{@code Dict}/{@code Slice} (no creation-time
 * tracking implemented), so those fall back to an imprecise result.
 */
public class SequenceLen extends UnaryExpression implements PluggableStatement {

	protected Statement st;

	protected SequenceLen(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression sequence) {
		super(cfg, location, constructName, sequence);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	public static SequenceLen build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new SequenceLen(cfg, location, "__len__", exprs[0]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);

		AnalysisState<A> result = state.bottom();
		boolean anyPointer = false;
		Set<Type> rts = analysis.getRuntimeTypesOf(state, expr, this);
		for (Type t : rts) {
			if (!t.isPointerType())
				continue;
			anyPointer = true;
			Type inner = t.asPointerType().getInnerType();

			if (inner.canBeAssignedTo(tupleType)) {
				HeapDereference deref = new HeapDereference(inner, expr, loc);
				Variable lenKey = new Variable(Int32Type.INSTANCE, "length", loc);
				AccessChild lenAccess = new AccessChild(Int32Type.INSTANCE, deref, lenKey, loc);
				result = result.lub(analysis.smallStepSemantics(state, lenAccess, st));
			} else
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Int32Type.INSTANCE, loc), st));
		}

		if (!anyPointer)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Int32Type.INSTANCE, loc), st));

		return result;
	}
}
