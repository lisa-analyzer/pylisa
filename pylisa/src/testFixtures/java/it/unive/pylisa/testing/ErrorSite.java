package it.unive.pylisa.testing;

/**
 * An error that may have been raised, with the call that raised it.
 *
 * @param type the name of the type of the error
 * @param call the name of the construct that raised it, such as
 *                 {@code mylib.Widget.update}, or {@code null} when the
 *                 analysis did not keep it (the error was smashed with others
 *                 of its type)
 * @param line the line of the raising call, or {@code -1} when it was not
 *                 kept
 */
public record ErrorSite(String type, String call, int line) {

	/**
	 * Yields whether the analysis kept the call that raised the error.
	 *
	 * @return {@code true} if it did
	 */
	public boolean hasSite() {
		return call != null;
	}
}
