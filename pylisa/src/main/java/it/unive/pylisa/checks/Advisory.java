package it.unive.pylisa.checks;

import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Objects;

/**
 * A statement that probably does not do what its author meant, found by a
 * check over the analysed code. An advisory claims nothing about the rest of
 * the program: it has no verdict, and its absence is not evidence that the
 * code is correct.
 *
 * @param kind     what the statement does not do
 * @param location where the statement is
 * @param message  a description of the statement, for people
 */
public record Advisory(Kind kind, CodeLocation location, String message) {

	/**
	 * What a statement found by an advisory check does not do.
	 */
	public enum Kind {

		/**
		 * A statement names a callable without calling it.
		 */
		CALLABLE_NOT_CALLED,

		/**
		 * A statement stores the result of a call that always returns
		 * {@code None}.
		 */
		STORED_NONE_RESULT
	}

	/**
	 * Builds the advisory.
	 *
	 * @param kind     what the statement does not do
	 * @param location where the statement is
	 * @param message  a description of the statement
	 */
	public Advisory {
		Objects.requireNonNull(kind);
		Objects.requireNonNull(location);
		Objects.requireNonNull(message);
	}
}
