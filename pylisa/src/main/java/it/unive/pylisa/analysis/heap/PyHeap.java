package it.unive.pylisa.analysis.heap;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.TypeHints;
import it.unive.pylisa.symbolic.operators.compare.IsInstance;
import it.unive.pylisa.symbolic.operators.compare.NotIsInstance;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

/**
 * The field-sensitive point-based heap, where reading a field holding objects
 * yields them (see {@link #process}), also refined by {@code isinstance} (see
 * {@link IsInstance}): in the branches of {@code isinstance(x, C)}, {@code x}
 * points only to the objects whose class satisfies {@code C}, or only to the
 * ones whose class does not, as its types are refined by the type domain (see
 * {@code PyInferredTypes}). Otherwise, the methods called on {@code x} would
 * also see the objects that its types exclude.
 */
public class PyHeap extends FieldSensitivePointBasedHeap {

	/**
	 * Assigns {@code rhs} to {@code id}. Reading a field holding objects (e.g.
	 * {@code o.x} in {@code y = o.x}) yields the location of the field: the
	 * identifier then points to the objects the field points to (otherwise it
	 * would point to nothing, as only pointers are stored).
	 */
	@Override
	protected HeapEnvWithFields process(
			HeapEnvWithFields state,
			Identifier id,
			SymbolicExpression rhs,
			ProgramPoint pp,
			SemanticOracle oracle,
			List<HeapReplacement> replacements,
			boolean rhsIsReceiver)
			throws SemanticException {
		// TODO move to LiSA: AllocationSiteBasedAnalysis.assign follows the
		// references held by the right-hand side only when the assigned
		// expression is itself an identifier (x = y). When it is a field access
		// (x = o.f), it is rewritten to the heap location of the field, that
		// process ignores as it is not a MemoryPointer: the assigned identifier
		// then references nothing, losing the object (reads through it, as x.v,
		// yield unknown values, or nothing for identifiers without the
		// fallback of rewriteHeapDereference). Here, a location that is tracked
		// by the heap (i.e. it holds references) is resolved to the
		// references it holds, as done for identifiers, and each of them is
		// processed as the right-hand side of the assignment.
		if (!(rhs instanceof AllocationSite) || !state.knowsIdentifier((Identifier) rhs))
			return super.process(state, id, rhs, pp, oracle, replacements, rhsIsReceiver);
		HeapEnvWithFields result = state.bottom();
		for (SymbolicExpression pointer : resolveIdentifier(state, (Identifier) rhs, pp))
			result = result.lub(super.process(state, id, pointer, pp, oracle, replacements, rhsIsReceiver));
		return result;
	}

	@Override
	public Pair<HeapEnvWithFields, List<HeapReplacement>> assume(
			HeapEnvWithFields state,
			SymbolicExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		Pair<HeapEnvWithFields, List<HeapReplacement>> result = super.assume(state, expression, src, dest, oracle);
		HeapEnvWithFields assumed = result.getLeft();
		// the expression is not rewritten yet (e.g. not o.x), so negations
		// are removed here
		SymbolicExpression test = expression;
		boolean negated = false;
		while (test instanceof UnaryExpression
				&& ((UnaryExpression) test).getOperator() == LogicalNegation.INSTANCE) {
			negated = !negated;
			test = ((UnaryExpression) test).getExpression();
		}
		if (assumed.isBottom() || assumed.isTop() || !(test instanceof BinaryExpression))
			return result;
		BinaryExpression binary = (BinaryExpression) test;
		if ((binary.getOperator() != IsInstance.INSTANCE && binary.getOperator() != NotIsInstance.INSTANCE)
				|| !(binary.getLeft() instanceof Identifier))
			return result;
		boolean positive = (binary.getOperator() == IsInstance.INSTANCE) != negated;
		Identifier id = (Identifier) binary.getLeft();
		if (!assumed.knowsIdentifier(id))
			return result;
		AllocationSites sites = assumed.getState(id);
		Set<Type> hints = oracle.getRuntimeTypesOf(binary.getRight(), src);
		if (sites.isTop() || sites.isBottom() || hints.isEmpty())
			return result;

		Set<AllocationSite> kept = new HashSet<>();
		for (AllocationSite site : sites.elements()) {
			Type object = new ReferenceType(site.getStaticType());
			for (Type hint : hints)
				if (TypeHints.isType(hint) != Satisfiability.NOT_SATISFIED && TypeHints.satisfies(object,
						hint) != (positive ? Satisfiability.NOT_SATISFIED : Satisfiability.SATISFIED)) {
					kept.add(site);
					break;
				}
		}
		if (kept.size() == sites.elements().size())
			return result;
		if (kept.isEmpty())
			return Pair.of(assumed.bottom(), result.getRight());
		return Pair.of(assumed.putState(id, new AllocationSites(kept)), result.getRight());
	}
}
