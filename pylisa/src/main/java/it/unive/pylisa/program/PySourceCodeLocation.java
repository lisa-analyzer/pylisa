package it.unive.pylisa.program;

import it.unive.lisa.program.SourceCodeLocation;

/**
 * The location of a construct in a Python source file. Its line and column
 * are those the analysis uses to tell constructs apart: the line where the
 * construct (or the part of it that the frontend builds a node for, such as
 * the argument list of a call) starts, and the column where it ends;
 * {@code x.f()} and {@code x.f().g()} start at the same column but end at
 * different ones. It also records where the whole construct starts, as
 * CPython reports it, for reports that point at it; that position takes no
 * part in equality or ordering.
 */
public class PySourceCodeLocation extends SourceCodeLocation {

	private final int startLine;

	private final int startCol;

	/**
	 * Builds the location.
	 *
	 * @param sourceFile the source file
	 * @param line       the line the analysis identifies the construct by,
	 *                       1-based
	 * @param col        the column where the construct ends, 0-based
	 * @param startLine  the line where the whole construct starts, 1-based
	 * @param startCol   the column where the whole construct starts, 0-based
	 */
	public PySourceCodeLocation(
			String sourceFile,
			int line,
			int col,
			int startLine,
			int startCol) {
		super(sourceFile, line, col);
		this.startLine = startLine;
		this.startCol = startCol;
	}

	/**
	 * Yields the line where the whole construct starts.
	 *
	 * @return the 1-based line
	 */
	public int getStartLine() {
		return startLine;
	}

	/**
	 * Yields the column where the whole construct starts, on
	 * {@link #getStartLine()}.
	 *
	 * @return the 0-based column
	 */
	public int getStartCol() {
		return startCol;
	}
}
