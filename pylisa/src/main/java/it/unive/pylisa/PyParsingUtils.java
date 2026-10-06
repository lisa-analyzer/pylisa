package it.unive.pylisa;

import it.unive.lisa.program.SourceCodeLocation;
import org.antlr.v4.runtime.ParserRuleContext;

/**
 * PyParsingUtils
 */
public class PyParsingUtils {

	public static int getLine(
			ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	public static int getCol(
			ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	public static SourceCodeLocation getLocation(
			String filePath,
			ParserRuleContext ctx) {
		return new SourceCodeLocation(filePath, getLine(ctx), getCol(ctx));
	}
}
