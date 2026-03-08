package it.unive.pylisa.frontend;

import org.antlr.v4.runtime.Token;

/**
 * Formats ANTLR syntax errors into human-readable strings.
 */
class SyntaxErrorFormatter {

	static String format(
			String sourceName,
			String source,
			Object offendingSymbol,
			int line,
			int charPositionInLine,
			String msg) {
		StringBuilder result = new StringBuilder();
		result.append("line ").append(line).append(":").append(charPositionInLine).append(" ").append(msg);

		if (offendingSymbol instanceof Token) {
			String text = ((Token) offendingSymbol).getText();
			if (text != null)
				result.append(" [offending token: '").append(text.replace("\n", "\\n").replace("\t", "\\t"))
						.append("']");
		}

		String sourceLine = getLine(source, line);
		if (sourceLine != null) {
			String printable = sourceLine.replace("\t", "\\t");
			int visualColumn = computeVisualColumn(sourceLine, charPositionInLine);
			result.append(System.lineSeparator()).append("  at ").append(sourceName).append(":").append(line)
					.append(System.lineSeparator()).append("  ").append(printable).append(System.lineSeparator())
					.append("  ").append(" ".repeat(Math.max(0, visualColumn))).append("^");

			if (hasMixedIndentation(sourceLine))
				result.append(System.lineSeparator())
						.append("  hint: mixed tabs and spaces in indentation can cause Python TabError-style failures");
		}

		return result.toString();
	}

	static String getLine(
			String source,
			int lineNumber) {
		if (source == null || lineNumber < 1)
			return null;

		String[] lines = source.split("\\r?\\n", -1);
		if (lineNumber > lines.length)
			return null;

		return lines[lineNumber - 1];
	}

	static int computeVisualColumn(
			String sourceLine,
			int column) {
		if (sourceLine == null || column <= 0)
			return 0;

		int visual = 0;
		for (int i = 0; i < sourceLine.length() && i < column; i++)
			visual += sourceLine.charAt(i) == '\t' ? 2 : 1;

		return visual;
	}

	static boolean hasMixedIndentation(
			String sourceLine) {
		if (sourceLine == null)
			return false;

		boolean sawSpaces = false;
		boolean sawTabs = false;
		for (int i = 0; i < sourceLine.length(); i++) {
			char c = sourceLine.charAt(i);
			if (c == ' ')
				sawSpaces = true;
			else if (c == '\t')
				sawTabs = true;
			else
				break;
		}

		return sawSpaces && sawTabs;
	}
}
