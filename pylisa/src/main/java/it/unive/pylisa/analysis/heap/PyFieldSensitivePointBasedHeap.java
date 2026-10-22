package it.unive.pylisa.analysis.heap;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.HeapDomain.HeapReplacement;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import java.util.List;

/**
 * A {@link FieldSensitivePointBasedHeap} that fixes some limitations of the
 * point-based heaps of LiSA that affect the analysis of Python programs. Each
 * fix is marked with a {@code TODO} comment, explaining why it is needed, so
 * that it can be moved to LiSA and removed from here.
 */
public class PyFieldSensitivePointBasedHeap extends FieldSensitivePointBasedHeap {

	/**
	 * {@inheritDoc}<br>
	 * <br>
	 * When the right-hand side of the assignment is a heap location that holds
	 * references (e.g. the field {@code f} in {@code x = o.f}, or the elements
	 * of a tuple or list in {@code a, b = t} and {@code x = l[0]}), the
	 * assigned identifier is made an alias of the objects referenced by the
	 * location.
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
		if (!(rhs instanceof MemoryPointer) && rhs instanceof AllocationSite && state.getKeys().contains(rhs)) {
			HeapEnvWithFields result = state.bottom();
			for (SymbolicExpression ref : resolveIdentifier(state, (Identifier) rhs, pp))
				result = result.lub(super.process(state, id, ref, pp, oracle, replacements, rhsIsReceiver));
			return result;
		}
		return super.process(state, id, rhs, pp, oracle, replacements, rhsIsReceiver);
	}
}
