package it.unive.pylisa;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.AllocationSites;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.lattices.GenericMapLattice;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import java.util.Collections;
import java.util.List;

public class PyFieldSensitivePointBasedHeap extends FieldSensitivePointBasedHeap {

	public PyFieldSensitivePointBasedHeap() {
		super();
	}

	private PyFieldSensitivePointBasedHeap(
			HeapEnvironment<AllocationSites> heapEnv,
			List<HeapReplacement> replacements,
			GenericMapLattice<AllocationSite, ExpressionSet> fields) {
		super(heapEnv, replacements, fields);
	}

	@Override
	public PyFieldSensitivePointBasedHeap popScope(
			ScopeToken scope)
			throws SemanticException {
		return this;
	}

	@Override
	public PyFieldSensitivePointBasedHeap pushScope(
			ScopeToken scope)
			throws SemanticException {
		return this;
	}

	@Override
	public FieldSensitivePointBasedHeap mk(
			FieldSensitivePointBasedHeap reference) {
		return new PyFieldSensitivePointBasedHeap(reference.heapEnv, Collections.emptyList(), reference.fields);
	}

	@Override
	public FieldSensitivePointBasedHeap mk(
			FieldSensitivePointBasedHeap reference,
			List<HeapReplacement> replacements) {
		return new PyFieldSensitivePointBasedHeap(reference.heapEnv, replacements, reference.fields);
	}

	@Override
	protected FieldSensitivePointBasedHeap mk(
			FieldSensitivePointBasedHeap reference,
			HeapEnvironment<AllocationSites> heapEnv) {
		return new PyFieldSensitivePointBasedHeap(heapEnv, Collections.emptyList(), reference.fields);
	}

}
