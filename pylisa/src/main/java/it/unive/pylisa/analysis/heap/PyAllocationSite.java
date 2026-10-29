package it.unive.pylisa.analysis.heap;

import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.HeapAllocationSite;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;

/**
 * The allocation sites of the objects in the heap (see {@link PyHeap}). As an
 * allocation site might be weak or strong, an allocation site with a field
 * might be surely initialized, if the field has been assigned on all the paths
 * reaching it (for a strong site), or maybe initialized otherwise (e.g. an
 * attribute assigned only in a branch, or on the objects of a weak site). The
 * initialization does not distinguish the sites: two sites with the same name
 * and weakness are the same site, whatever their initialization. Since the
 * sites with fields are created when the fields are accessed, the
 * initialization is computed from the heap (see
 * {@link PyHeap#surelyInitialized}).
 */
public class PyAllocationSite extends HeapAllocationSite {

	private final boolean initialized;

	/**
	 * Builds an allocation site without field.
	 *
	 * @param staticType   the static type of the site
	 * @param locationName the name of the location of the site
	 * @param isWeak       whether the site is weak
	 * @param location     the code location of the site
	 */
	public PyAllocationSite(
			Type staticType,
			String locationName,
			boolean isWeak,
			CodeLocation location) {
		this(staticType, locationName, (String) null, null, isWeak, true, location);
	}

	/**
	 * Builds an allocation site with a field.
	 *
	 * @param staticType   the static type of the site
	 * @param locationName the name of the location of the site
	 * @param field        the field, or {@code null}
	 * @param context      the interprocedural context where the site has been
	 *                         allocated, or {@code null}
	 * @param isWeak       whether the site is weak
	 * @param initialized  whether the field is surely initialized
	 * @param location     the code location of the site
	 */
	public PyAllocationSite(
			Type staticType,
			String locationName,
			String field,
			ScopeId<?> context,
			boolean isWeak,
			boolean initialized,
			CodeLocation location) {
		super(staticType, locationName, field, context, isWeak, location);
		this.initialized = initialized;
	}

	/**
	 * Yields the allocation site of pylisa corresponding to the given one, with
	 * the same annotations, and the given initialization.
	 *
	 * @param site        the site
	 * @param initialized whether the field of the site is surely initialized
	 *
	 * @return the site
	 */
	public static PyAllocationSite of(
			AllocationSite site,
			boolean initialized) {
		PyAllocationSite result = new PyAllocationSite(site.getStaticType(), site.getLocationName(), site.getField(),
				site.getContext(), site.isWeak(), initialized, site.getCodeLocation());
		result.setAllocation(site.isAllocation());
		for (Annotation ann : site.getAnnotations())
			result.addAnnotation(ann);
		return result;
	}

	/**
	 * Yields whether the field of this site is surely initialized (always true
	 * for the sites without field).
	 *
	 * @return whether the field is surely initialized
	 */
	public boolean isSurelyInitialized() {
		return initialized;
	}

	@Override
	public PyAllocationSite toWeak() {
		return isWeak() ? this : copy(getStaticType(), getField(), true, initialized);
	}

	@Override
	public PyAllocationSite withField(
			SymbolicExpression field) {
		if (getField() != null)
			throw new IllegalStateException("Cannot add a field to an allocation site that already has one");
		return copy(getStaticType(), field.toString(), isWeak(), true);
	}

	@Override
	public PyAllocationSite withType(
			Type type) {
		return copy(type, getField(), isWeak(), initialized);
	}

	@Override
	public PyAllocationSite withoutField() {
		if (getField() == null)
			return this;
		return copy(getStaticType(), null, isWeak(), true);
	}

	@Override
	public PyAllocationSite withContext(
			ScopeId<?> context) {
		return new PyAllocationSite(getStaticType(), getLocationName(), getField(), context, isWeak(), initialized,
				getCodeLocation());
	}

	@Override
	public PyAllocationSite asNonAllocation() {
		if (!isAllocation())
			return this;
		return copy(getStaticType(), getField(), isWeak(), initialized);
	}

	private PyAllocationSite copy(
			Type type,
			String field,
			boolean weak,
			boolean init) {
		// as in HeapAllocationSite, the annotations and the allocation flag are
		// not copied
		return new PyAllocationSite(type, getLocationName(), field, getContext(), weak, init, getCodeLocation());
	}
}
