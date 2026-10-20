package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Objects;

/**
 * A location derived from another one, e.g. the one of the list built by
 * {@code lst.copy()}: since memory allocations are identified by their
 * location, the list must be allocated at a location different from the one of
 * the call (that is also the one of the list literal in {@code [1, 2].copy()}).
 */
public class DerivedLocation implements CodeLocation {

	private final CodeLocation base;

	private final String tag;

	/**
	 * Builds the location.
	 *
	 * @param base the location this one is derived from
	 * @param tag  what distinguishes this location from {@code base}
	 */
	public DerivedLocation(
			CodeLocation base,
			String tag) {
		this.base = base;
		this.tag = tag;
	}

	@Override
	public String getCodeLocation() {
		return base.getCodeLocation() + "#" + tag;
	}

	@Override
	public int compareTo(
			CodeLocation other) {
		return getCodeLocation().compareTo(other.getCodeLocation());
	}

	@Override
	public boolean equals(
			Object obj) {
		return obj instanceof DerivedLocation && base.equals(((DerivedLocation) obj).base)
				&& tag.equals(((DerivedLocation) obj).tag);
	}

	@Override
	public int hashCode() {
		return Objects.hash(base, tag);
	}

	@Override
	public String toString() {
		return getCodeLocation();
	}
}
