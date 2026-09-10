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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
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

		Type dereferencedType = null;
		Set<Type> rts = analysis.getRuntimeTypesOf(state, left, this);
		for (Type t : rts)
			if (t.isPointerType()) {
				Type inner = t.asPointerType().getInnerType();
				dereferencedType = dereferencedType == null ? inner : dereferencedType.commonSupertype(inner);
			}
		if (dereferencedType == null)
			dereferencedType = Untyped.INSTANCE;

		HeapDereference deref = new HeapDereference(dereferencedType, left, loc);
		AccessChild access = new AccessChild(Untyped.INSTANCE, deref, right, loc);
		return analysis.smallStepSemantics(state, access, st);
	}
}