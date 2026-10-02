package it.unive.pylisa.program;

import it.unive.lisa.program.cfg.CodeLocation;

/**
 * The location of code that pylisa builds itself and that has no place in a
 * source file: module and class initializers of library specifications, the
 * program's entry point, synthetic imports and calls. Unlike LiSA's
 * {@code SyntheticLocation}, it prints and hashes the same way in every run,
 * so that contexts built on synthetic calls, and the result files named after
 * them, are identical across runs.
 */
public final class PySyntheticLocation implements CodeLocation {

	/**
	 * The only synthetic location.
	 */
	public static final PySyntheticLocation INSTANCE = new PySyntheticLocation();

	private static final String TEXT = "<synthetic>";

	private PySyntheticLocation() {
	}

	@Override
	public String getCodeLocation() {
		return TEXT;
	}

	/**
	 * Compares this location with another one: it is equal to itself and
	 * follows every other location, since a source location precedes any
	 * location that is not one.
	 */
	@Override
	public int compareTo(
			CodeLocation other) {
		return other == this ? 0 : 1;
	}

	@Override
	public int hashCode() {
		return TEXT.hashCode();
	}

	@Override
	public boolean equals(
			Object obj) {
		return obj == this;
	}

	@Override
	public String toString() {
		return TEXT;
	}
}
