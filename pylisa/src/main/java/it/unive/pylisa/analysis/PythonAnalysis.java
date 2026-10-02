package it.unive.pylisa.analysis;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.LiSA;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.ModuleProvider;
import it.unive.pylisa.frontend.PyFrontend;
import it.unive.pylisa.program.ProgramSettings;
import java.io.IOException;
import java.util.List;

/**
 * The one way to analyse a Python program with pylisa: it translates the
 * program, configures LiSA for a value configuration and runs it with the
 * given semantic checks, which read the results. Command-line tools and tests
 * use it alike.
 */
public final class PythonAnalysis {

	private PythonAnalysis() {
	}

	/**
	 * Analyses a program.
	 *
	 * @param program     the path of the Python entry file
	 * @param config      the value configuration
	 * @param workdir     the working directory of the run, where LiSA writes
	 *                        its outputs
	 * @param dumpResults whether LiSA's results are dumped under the working
	 *                        directory
	 * @param checks      the semantic checks to run on the results; the
	 *                        caller keeps them to read what they found
	 * @param providers   the providers of modules the program imports that
	 *                        are neither in the program nor in the library
	 *                        specifications
	 * @param settings    the settings of the environment the program runs
	 *                        in, which the translated program carries and
	 *                        library models read
	 *
	 * @throws IOException            if the program cannot be read
	 * @throws AnalysisSetupException if the program cannot be translated
	 * @throws AnalysisException      if the analysis fails
	 */
	public static void run(
			String program,
			AnalysisConfig config,
			String workdir,
			boolean dumpResults,
			List<? extends SemanticCheck<?, ?>> checks,
			List<? extends ModuleProvider> providers,
			ProgramSettings settings)
			throws IOException,
			AnalysisSetupException,
			AnalysisException {
		Program translated = translate(program, providers, settings);
		LiSAConfiguration conf = config.configuration(workdir, dumpResults);
		conf.semanticChecks.addAll(checks);
		new LiSA(conf).run(translated);
	}

	/**
	 * Translates a program into the LiSA program that {@link #run} analyses,
	 * for callers that run LiSA themselves on a configuration set up by
	 * {@link AnalysisConfig#configure}.
	 *
	 * @param program   the path of the Python entry file
	 * @param providers the providers of modules the program imports that are
	 *                      neither in the program nor in the library
	 *                      specifications
	 * @param settings  the settings of the environment the program runs in
	 *
	 * @return the translated program
	 *
	 * @throws IOException            if the program cannot be read
	 * @throws AnalysisSetupException if the program cannot be translated
	 */
	public static Program translate(
			String program,
			List<? extends ModuleProvider> providers,
			ProgramSettings settings)
			throws IOException,
			AnalysisSetupException {
		PyFrontend frontend = new PyFrontend(program, settings);
		for (ModuleProvider provider : providers)
			frontend.addModuleProvider(provider);
		return frontend.toLiSAProgram(true);
	}
}
