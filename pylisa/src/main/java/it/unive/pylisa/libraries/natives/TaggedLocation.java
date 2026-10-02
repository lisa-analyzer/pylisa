package it.unive.pylisa.libraries.natives;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Objects;

/**
 * A location derived from another one by a tag, used to give distinct
 * allocation sites to the several objects that a single library call creates
 * (for instance, the several helper objects one constructor creates).
 * Two tagged locations are equal when both their base location and their tag
 * are.
 * <p>
 * A tagged location is a source position: the position of its base when the
 * base is one, so that consumers that read positions see where the call that
 * created the object is; its name keeps the tag. A base that is not a source
 * position gives the position line and column -1 in a file named after the
 * base.
 * </p>
 */
public final class TaggedLocation extends SourceCodeLocation {

	private final CodeLocation base;

	private final String tag;

	/**
	 * Builds the location.
	 *
	 * @param base the location it derives from
	 * @param tag  what distinguishes it from other locations with the same
	 *                 base
	 */
	public TaggedLocation(
			CodeLocation base,
			String tag) {
		super(sourceFile(base), base instanceof SourceCodeLocation at ? at.getLine() : -1,
				base instanceof SourceCodeLocation at ? at.getCol() : -1);
		this.base = base;
		this.tag = Objects.requireNonNull(tag);
	}

	private static String sourceFile(
			CodeLocation base) {
		Objects.requireNonNull(base);
		return base instanceof SourceCodeLocation at ? at.getSourceFile() : base.getCodeLocation();
	}

	@Override
	public String getCodeLocation() {
		return base.getCodeLocation() + "#" + tag;
	}

	/**
	 * Orders by position, then by tag; a tagged location follows the plain
	 * position it shares. LiSA's own {@link SourceCodeLocation#compareTo}
	 * compares positions only, so a plain location finds itself equal to a
	 * tagged one at its position: no ordering over both is consistent with
	 * {@code equals} on that side.
	 */
	@Override
	public int compareTo(
			CodeLocation other) {
		if (!(other instanceof SourceCodeLocation at))
			return getCodeLocation().compareTo(other.getCodeLocation());
		int byPosition = super.compareTo(at);
		if (byPosition != 0)
			return byPosition;
		return other instanceof TaggedLocation tagged ? tag.compareTo(tagged.tag) : 1;
	}

	@Override
	public boolean equals(
			Object other) {
		return other instanceof TaggedLocation location && base.equals(location.base) && tag.equals(location.tag);
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
