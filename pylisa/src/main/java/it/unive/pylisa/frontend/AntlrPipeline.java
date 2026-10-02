package it.unive.pylisa.frontend;

import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.File_inputContext;
import java.io.IOException;
import java.util.Objects;
import org.antlr.v4.runtime.BailErrorStrategy;
import org.antlr.v4.runtime.BaseErrorListener;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.misc.ParseCancellationException;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Strict ANTLR pipeline for a single Python source: lexer + parser wired with
 * {@link SyntaxErrorFormatter} as the sole error listener and
 * {@link BailErrorStrategy} so the first parse error aborts the run. Any syntax
 * error surfaces as an {@link IOException} whose message is the formatted
 * location + snippet.
 * <p>
 * Extracted from {@code PyFrontend.parseFileInputStrict} in Chunk 7 of the
 * front-end refactor so the entry point holds only orchestration.
 */
public final class AntlrPipeline {

	private static final Logger LOG = LogManager.getLogger(AntlrPipeline.class);

	private final String sourceName;
	private final String source;

	/**
	 * @param sourceName the file path (or logical name) to report in syntax
	 *                       errors
	 * @param source     the raw Python source text to lex and parse
	 */
	public AntlrPipeline(
			String sourceName,
			String source) {
		this.sourceName = Objects.requireNonNull(sourceName);
		this.source = Objects.requireNonNull(source);
	}

	/**
	 * Lexes and parses {@link #source}, returning the {@code file_input} parse
	 * tree. Throws {@link IOException} wrapping a
	 * {@link SyntaxErrorFormatter}-formatted message on the first parse error.
	 */
	public File_inputContext parseFile() throws IOException {
		LOG.debug("lexing {}", sourceName);
		Python3Lexer lexer = new Python3Lexer(CharStreams.fromString(source, sourceName));
		BaseErrorListener listener = new StrictErrorListener(sourceName, source);
		lexer.removeErrorListeners();
		lexer.addErrorListener(listener);

		LOG.debug("parsing {}", sourceName);
		Python3Parser parser = new Python3Parser(new CommonTokenStream(lexer));
		parser.removeErrorListeners();
		parser.addErrorListener(listener);
		parser.setErrorHandler(new BailErrorStrategy());

		try {
			return parser.file_input();
		} catch (ParseCancellationException ex) {
			throw new IOException("Invalid Python input in '" + sourceName + "': "
					+ extractDetail(ex), ex);
		}
	}

	private String extractDetail(
			ParseCancellationException ex) {
		String detail = ex.getMessage();
		if (detail != null)
			return detail;
		if (ex.getCause() instanceof RecognitionException re) {
			Token tok = re.getOffendingToken();
			if (tok != null)
				return SyntaxErrorFormatter.format(
						sourceName,
						source,
						tok,
						tok.getLine(),
						tok.getCharPositionInLine(),
						"unexpected token '" + tok.getText() + "'");
		}
		return "<no detail>";
	}

	/**
	 * Fails fast on the first lexer/parser diagnostic: re-throws a
	 * {@link ParseCancellationException} carrying a
	 * {@link SyntaxErrorFormatter}-formatted message.
	 */
	private static final class StrictErrorListener extends BaseErrorListener {
		private final String sourceName;
		private final String source;

		StrictErrorListener(
				String sourceName,
				String source) {
			this.sourceName = sourceName;
			this.source = source;
		}

		@Override
		public void syntaxError(
				Recognizer<?, ?> recognizer,
				Object offendingSymbol,
				int line,
				int charPositionInLine,
				String msg,
				RecognitionException e) {
			throw new ParseCancellationException(
					SyntaxErrorFormatter.format(sourceName, source, offendingSymbol, line, charPositionInLine, msg));
		}
	}
}
