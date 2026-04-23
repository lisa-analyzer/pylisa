package it.unive.pylisa.frontend;

import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.antlr.v4.runtime.BaseErrorListener;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;
import org.junit.Test;

/**
 * Walks every .py file under the dispatch test bundle, parses each with the
 * current ANTLR grammar using a non-bailing error listener, and writes a
 * human-readable summary of every FIRST syntax error per file (since the
 * parser recovers clumsily, only the first error per file is the root cause;
 * trailing errors are recovery noise).
 *
 * Diagnostic only — never fails the build.
 */
public class DispatchSyntaxScanTest {

	private static final Path DISPATCH_ROOT = Path.of(
			"py-testcases/microservices/evaluation/dispatch");

	private static final Pattern WALRUS = Pattern.compile(":=");
	private static final Pattern MATCH_STMT = Pattern.compile("^\\s*match\\s+.*:\\s*$");
	private static final Pattern PAREN_WITH = Pattern.compile("^\\s*with\\s*\\(");
	private static final Pattern UNION_PIPE_TYPE = Pattern.compile("[A-Za-z_][\\w.]*\\s*\\|\\s*[A-Za-z_]");
	private static final Pattern POS_ONLY = Pattern.compile("\\(.*,\\s*/\\s*[,)]");
	private static final Pattern TYPE_ALIAS = Pattern.compile("^\\s*type\\s+\\w+\\s*=");
	private static final Pattern EXCEPT_STAR = Pattern.compile("^\\s*except\\*");

	@Test
	public void scanDispatchForSyntaxErrors() throws IOException {
		if (!Files.isDirectory(DISPATCH_ROOT)) {
			System.out.println("[skip] dispatch root not found: " + DISPATCH_ROOT.toAbsolutePath());
			return;
		}

		List<Path> files;
		try (Stream<Path> s = Files.walk(DISPATCH_ROOT)) {
			files = s.filter(Files::isRegularFile)
					.filter(p -> p.toString().endsWith(".py"))
					.sorted()
					.toList();
		}

		List<FileReport> reports = new ArrayList<>();
		Map<String, Integer> featureHistogram = new LinkedHashMap<>();
		int ok = 0;

		for (Path file : files) {
			FileReport r = parseOne(file);
			if (r.firstError == null) {
				ok++;
			} else {
				reports.add(r);
				featureHistogram.merge(r.feature, 1, Integer::sum);
			}
		}

		StringBuilder out = new StringBuilder();
		out.append("\n==================== DISPATCH SYNTAX SCAN ====================\n");
		out.append("Scanned files : ").append(files.size()).append('\n');
		out.append("Clean         : ").append(ok).append('\n');
		out.append("With errors   : ").append(reports.size()).append('\n');
		out.append("===============================================================\n\n");
		out.append("Missing feature by file count:\n");
		featureHistogram.entrySet().stream()
				.sorted(Map.Entry.<String, Integer>comparingByValue(Comparator.reverseOrder()))
				.forEach(e -> out.append("  ").append(pad(e.getValue() + "x", 5)).append(e.getKey()).append('\n'));

		out.append("\n===============================================================\n");
		out.append("File-by-file first-error (root cause)\n");
		out.append("===============================================================\n");

		reports.sort(Comparator.comparing(r -> r.feature + "/" + r.path));
		for (FileReport r : reports) {
			Path rel = DISPATCH_ROOT.relativize(r.path);
			out.append('\n').append('[').append(r.feature).append("]  ").append(rel).append('\n');
			out.append("  line ").append(r.firstError.line).append(": ");
			out.append(r.sourceLine == null ? "<line unavailable>" : r.sourceLine.stripTrailing()).append('\n');
			out.append("  parser: ").append(truncate(r.firstError.msg, 140)).append('\n');
			out.append("  totalErrorsInFile: ").append(r.totalErrors).append('\n');
		}
		out.append("===============================================================\n");

		System.out.println(out);
		Path report = DISPATCH_ROOT.resolve("syntax-scan-report.txt");
		Files.writeString(report, out.toString());
		System.out.println("Report: " + report.toAbsolutePath());
	}

	private static FileReport parseOne(Path file) {
		FileReport r = new FileReport();
		r.path = file;
		String source;
		try {
			source = SourceReader.readNormalizedFile(file.toString());
		} catch (IOException io) {
			r.firstError = new Err(0, 0, "io: " + io.getMessage());
			r.totalErrors = 1;
			r.feature = "io-error";
			return r;
		}

		String[] lines = source.split("\n", -1);
		List<Err> errs = new ArrayList<>();
		Python3Lexer lexer = new Python3Lexer(CharStreams.fromString(source));
		CollectingListener listener = new CollectingListener(errs);
		lexer.removeErrorListeners();
		lexer.addErrorListener(listener);

		Python3Parser parser = new Python3Parser(new CommonTokenStream(lexer));
		parser.removeErrorListeners();
		parser.addErrorListener(listener);

		try {
			parser.file_input();
		} catch (RuntimeException ex) {
			errs.add(new Err(0, 0, "parser-crash: " + ex.getClass().getSimpleName()));
		}
		r.totalErrors = errs.size();
		if (!errs.isEmpty()) {
			r.firstError = errs.get(0);
			int idx = r.firstError.line - 1;
			if (idx >= 0 && idx < lines.length) {
				r.sourceLine = lines[idx];
				r.feature = classifyBySource(r.sourceLine, lines, idx);
			} else {
				r.feature = "unknown";
			}
		}
		return r;
	}

	private static String classifyBySource(String line, String[] allLines, int idx) {
		if (line == null) return "unknown";
		if (WALRUS.matcher(line).find()) return "walrus-:=";
		if (MATCH_STMT.matcher(line).find()) return "match-case";
		if (EXCEPT_STAR.matcher(line).find()) return "except*";
		if (TYPE_ALIAS.matcher(line).find()) return "type-alias-stmt (PEP 695)";
		if (PAREN_WITH.matcher(line).find()) return "parenthesised-with (PEP 617)";
		if (POS_ONLY.matcher(line).find()) return "positional-only-/";
		if (UNION_PIPE_TYPE.matcher(line).find() && line.contains(":")) return "union-| in types (PEP 604)";
		// Look a few lines back for a preceding walrus/match in case the error
		// actually is a recovery-delayed report of a previous-line construct.
		for (int back = 1; back <= 3 && idx - back >= 0; back++) {
			String prev = allLines[idx - back];
			if (WALRUS.matcher(prev).find()) return "walrus-:=";
			if (MATCH_STMT.matcher(prev).find()) return "match-case";
			if (PAREN_WITH.matcher(prev).find()) return "parenthesised-with (PEP 617)";
		}
		return "unclassified";
	}

	private static String pad(String s, int w) {
		return s.length() >= w ? s + " " : s + " ".repeat(w - s.length());
	}

	private static String truncate(String s, int n) {
		if (s == null) return "";
		s = s.replace('\n', ' ').replace('\r', ' ');
		return s.length() <= n ? s : s.substring(0, n) + "...";
	}

	private static final class FileReport {
		Path path;
		Err firstError;
		int totalErrors;
		String sourceLine;
		String feature = "unclassified";
	}

	private record Err(int line, int col, String msg) {}

	private static final class CollectingListener extends BaseErrorListener {
		private final List<Err> errs;

		CollectingListener(List<Err> errs) {
			this.errs = errs;
		}

		@Override
		public void syntaxError(
				Recognizer<?, ?> recognizer,
				Object offendingSymbol,
				int line,
				int charPositionInLine,
				String msg,
				RecognitionException e) {
			errs.add(new Err(line, charPositionInLine, msg));
		}
	}
}
