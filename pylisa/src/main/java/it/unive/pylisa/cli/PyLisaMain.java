package it.unive.pylisa.cli;

import it.unive.lisa.LiSA;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.PythonAnalysis;
import it.unive.pylisa.checks.Advisory;
import it.unive.pylisa.checks.AlwaysRaisesCheck;
import it.unive.pylisa.checks.AssertChecker;
import it.unive.pylisa.checks.AssertionVerdict;
import it.unive.pylisa.checks.CallableNotCalledCheck;
import it.unive.pylisa.checks.StoredNoneResultCheck;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.ProgramSettings;
import it.unive.pylisa.program.PySourceCodeLocation;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;

/**
 * pylisa's command line: analyses a Python program, which needs no library
 * model beyond pylisa's own, and prints what pylisa's generic checks find.
 * <p>
 * Usage: {@code pylisa [--config CP|CP_BSS] [--workdir <dir>] <file.py>}. The
 * analysis is the one of {@link AnalysisConfig}, constant propagation with
 * bounded string sets by default; it is best-effort, since calls to unknown
 * code are assumed to return an unknown value and to have no other effect.
 * LiSA writes its outputs to the working directory, {@code pylisa-output}
 * under the current directory by default. The report lists the statements
 * that always raise, the verdicts of the assertions, the advisories, and the
 * constructs of the program that the frontend translates approximately, which
 * the results may not hold for.
 * </p>
 * <p>
 * The exit status is 0 when no statement always raises and no assertion
 * fails, 1 otherwise, and 2 when the command line is wrong.
 * </p>
 */
public final class PyLisaMain {

	/**
	 * How the command is used.
	 */
	public static final String USAGE = "usage: pylisa [--config CP|CP_BSS] [--workdir <dir>] <file.py>";

	private PyLisaMain() {
	}

	/**
	 * Runs the command and exits with its status.
	 *
	 * @param args the command-line arguments
	 *
	 * @throws Exception if the program cannot be read, translated or analysed
	 */
	public static void main(
			String[] args)
			throws Exception {
		System.exit(run(args, System.out, System.err));
	}

	/**
	 * Runs the command.
	 *
	 * @param args the command-line arguments
	 * @param out  where the report is printed
	 * @param err  where usage errors are printed
	 *
	 * @return the exit status
	 *
	 * @throws Exception if the program cannot be read, translated or analysed
	 */
	public static int run(
			String[] args,
			PrintStream out,
			PrintStream err)
			throws Exception {
		AnalysisConfig config = AnalysisConfig.CP_BSS;
		Path workdir = Path.of("pylisa-output");
		String file = null;
		for (int i = 0; i < args.length; i++)
			switch (args[i]) {
			case "--config" -> {
				if (++i == args.length || !isConfig(args[i]))
					return usage(err, "--config takes one of CP, CP_BSS");
				config = AnalysisConfig.valueOf(args[i].toUpperCase(Locale.ROOT));
			}
			case "--workdir" -> {
				if (++i == args.length)
					return usage(err, "--workdir takes a directory");
				workdir = Path.of(args[i]);
			}
			default -> {
				if (args[i].startsWith("--") || file != null)
					return usage(err, "unexpected argument " + args[i]);
				file = args[i];
			}
			}
		if (file == null)
			return usage(err, "no program given");
		if (!Files.isRegularFile(Path.of(file)))
			return usage(err, "no such file: " + file);

		Program program = PythonAnalysis.translate(file, List.of(), ProgramSettings.NONE);
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = workdir.toString();
		config.configure(conf);
		AlwaysRaisesCheck<?, ?> raises = new AlwaysRaisesCheck<>();
		AssertChecker<?, ?> assertions = new AssertChecker<>();
		CallableNotCalledCheck<?, ?> notCalled = new CallableNotCalledCheck<>();
		StoredNoneResultCheck<?, ?> storedNone = new StoredNoneResultCheck<>();
		conf.semanticChecks.add(raises);
		conf.semanticChecks.add(assertions);
		conf.semanticChecks.add(notCalled);
		conf.semanticChecks.add(storedNone);
		new LiSA(conf).run(program);

		out.println("pylisa " + file + " (" + config.label()
				+ ": calls to unknown code return an unknown value and have no other effect)");
		if (raises.sawUnsoundTranslation() || assertions.sawUnsoundTranslation() || notCalled.sawUnsoundTranslation()
				|| storedNone.sawUnsoundTranslation())
			out.println("best-effort: part of the program is translated unsoundly or approximately (listed below),"
					+ " so the findings may not hold");
		List<String> raising = new ArrayList<>();
		for (AlwaysRaisesCheck.Finding finding : raises.getFindings())
			raising.add(position(finding.statement().getLocation())
					+ ": every execution that completes raises an exception, such as " + finding.exceptions());
		section(out, "Always raises", raising);
		List<String> verdicts = new ArrayList<>();
		boolean fails = false;
		for (Map.Entry<CodeLocation, AssertionVerdict> verdict : assertions.getVerdicts().entrySet()) {
			verdicts.add(position(verdict.getKey()) + ": " + verdict.getValue());
			fails |= verdict.getValue() == AssertionVerdict.FAILS;
		}
		section(out, "Assertions", verdicts);
		List<String> advisories = new ArrayList<>();
		for (Advisory advisory : notCalled.advisories())
			advisories.add(position(advisory.location()) + ": " + advisory.message());
		for (Advisory advisory : storedNone.advisories())
			advisories.add(position(advisory.location()) + ": " + advisory.message());
		section(out, "Advisories", advisories);
		section(out, "Translated unsoundly (results may not hold)",
				new ArrayList<>(marks(program, ParserSupport.UNSOUND_TRANSLATION)));
		section(out, "Known frontend limitations", new ArrayList<>(marks(program, ParserSupport.KNOWN_LIMITATION)));
		return raises.getFindings().isEmpty() && !fails ? 0 : 1;
	}

	/**
	 * Yields where a construct starts, as {@code file:line:column}; pylisa's
	 * locations otherwise print the column where it ends.
	 */
	private static String position(
			CodeLocation location) {
		return location instanceof PySourceCodeLocation python
				? python.getSourceFile() + ":" + python.getStartLine() + ":" + python.getStartCol()
				: location.toString();
	}

	private static boolean isConfig(
			String name) {
		for (AnalysisConfig config : AnalysisConfig.values())
			if (config.name().equalsIgnoreCase(name))
				return true;
		return false;
	}

	private static int usage(
			PrintStream err,
			String problem) {
		err.println("pylisa: " + problem);
		err.println(USAGE);
		return 2;
	}

	private static void section(
			PrintStream out,
			String title,
			List<String> lines) {
		out.println(title + ": " + (lines.isEmpty() ? "none" : lines.size()));
		lines.stream().sorted().forEach(line -> out.println("  " + line));
	}

	/**
	 * Yields the functions of a program that carry a mark of the frontend,
	 * with the construct each mark names.
	 */
	private static Set<String> marks(
			Program program,
			String kind) {
		Set<String> marks = new TreeSet<>();
		for (CodeMember member : program.getCodeMembersRecursively())
			for (Annotation annotation : member.getDescriptor().getAnnotations())
				if (annotation.getAnnotationName().equals(kind))
					for (AnnotationMember field : annotation.getAnnotationMembers())
						if (field.getId().equals(ParserSupport.CONSTRUCT))
							marks.add(member.getDescriptor().getFullName() + ": " + field.getValue());
		return marks;
	}
}
