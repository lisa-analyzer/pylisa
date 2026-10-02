package it.unive.pylisa.symbolic;

import java.util.Objects;

/**
 * Marker value type for analyzer-internal heap-cell names.
 * <p>
 * The PyLiSA heap stores per-allocation cells keyed by the {@code toString()}
 * of the {@code AccessChild} child expression. Lists and (eventually) other
 * containers need a small set of <em>internal</em> cells — currently just
 * {@code $size} — that the lowering writes/reads as instrumentation. If those
 * cells were keyed by ordinary {@code Constant}s of {@code String} type, a
 * user-level access like {@code m["$size"] = 100} on a future map domain
 * would resolve to the same {@link it.unive.lisa.lattices.heap.allocations.AllocationSite}
 * identifier and overwrite our metadata.
 * <p>
 * To prevent that collision by construction we wrap internal names in this
 * class and pass them as the <em>value</em> of a {@code Constant}. Two
 * properties combine to make the resulting cell key disjoint from any
 * user-reachable string key:
 * <ol>
 * <li>{@code Constant.toString()} only quotes its value when the value's
 * Java type is {@code String}. With an {@link InternalAttribute} value, the
 * rendering goes through {@link #toString()} unchanged — so an internal
 * {@code size} cell renders as {@code [->size]} while a user key
 * {@code m["->size"]} still renders as {@code ["->size"]} (quoted). The
 * presence or absence of quotes distinguishes the two namespaces.</li>
 * <li>The {@code ->} prefix is a visual marker (mirroring the C-style member
 * access used by CPython for fields like {@code ob_size}) and gives a quick
 * read of "this is internal metadata, not user data".</li>
 * </ol>
 *
 * <p>
 * This class is intentionally minimal. When dict/map support lands we may
 * want a heavier mechanism (a dedicated {@code AllocationSite} subclass that
 * renders without brackets, e.g. {@code pp@loc->size}); the current
 * {@code [->size]} form is already collision-safe so the rendering tweak is
 * cosmetic.
 */
public final class InternalAttribute {

	private final String name;

	public InternalAttribute(
			String name) {
		this.name = Objects.requireNonNull(name);
	}

	public String getName() {
		return name;
	}

	@Override
	public String toString() {
		return "->" + name;
	}

	@Override
	public int hashCode() {
		return name.hashCode();
	}

	@Override
	public boolean equals(
			Object o) {
		return o instanceof InternalAttribute && ((InternalAttribute) o).name.equals(name);
	}
}
