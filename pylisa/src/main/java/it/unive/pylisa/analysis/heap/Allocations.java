package it.unive.pylisa.analysis.heap;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.heap.MemoryAllocation;

/**
 * The allocations of objects in the heap. An object allocated where another one
 * has already been allocated (e.g. by a second call to the same function) is
 * represented, together with the other one, by a weak allocation site (a
 * summary). The heap turns the strong site into the weak one for all the
 * references to it only when the allocation itself is evaluated (see
 * {@code FieldSensitivePointBasedHeap}): assigning a reference to the
 * allocation (e.g. {@code x = HeapReference(MemoryAllocation)}) would only make
 * the assigned identifier point to the weak site, while the other references
 * would keep pointing to the strong one, whose values are then lost when the
 * two are joined (as a weak site replaces the strong one with the same name).
 */
public final class Allocations {

	private Allocations() {
	}

	/**
	 * Evaluates the given allocation, before a reference to it is assigned or
	 * computed: the objects already allocated at its location become a weak
	 * site, for all the references to them. The computed expressions of the
	 * state are not changed.
	 *
	 * @param <A>      the kind of lattice of the analysis
	 * @param <D>      the kind of domain of the analysis
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the state where the object is allocated
	 * @param alloc    the allocation
	 * @param pp       the program point allocating the object
	 *
	 * @return the state where the allocation has been evaluated
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> allocate(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			MemoryAllocation alloc,
			ProgramPoint pp)
			throws SemanticException {
		return analysis.smallStepSemantics(state, alloc, context, pp)
				.withExecutionExpressions(state.getExecutionExpressions());
	}
}
