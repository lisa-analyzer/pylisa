package it.unive.pylisa;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSites;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;

public class PyFieldSensitivePointBasedHeap extends FieldSensitivePointBasedHeap {

	public PyFieldSensitivePointBasedHeap() {
		super();
	}

	private PyFieldSensitivePointBasedHeap(HeapEnvironment<AllocationSites> allocationSites) {
		super(allocationSites);
	}

	@Override
	public PyFieldSensitivePointBasedHeap from(PointBasedHeap original) {
		return new PyFieldSensitivePointBasedHeap(original.heapEnv);
	}

	@Override
	public PyFieldSensitivePointBasedHeap popScope(ScopeToken scope) throws SemanticException {
		return this;
	}

	@Override
	public PyFieldSensitivePointBasedHeap pushScope(ScopeToken scope) throws SemanticException {
		return this;
	}
}
