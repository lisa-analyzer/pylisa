package it.unive.pylisa;

import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAReport;
import it.unive.lisa.analysis.ConstantValuePropagation;
import it.unive.lisa.analysis.Reachability;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.inlining.InliningAnalysis;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.listeners.BottomTopListener;
import it.unive.lisa.listeners.CallResolutionListener;
import it.unive.lisa.outputs.HtmlResults;
import it.unive.lisa.outputs.JSONReportDumper;
import it.unive.lisa.outputs.JSONResults;
import it.unive.lisa.outputs.messages.Message;
import it.unive.lisa.program.Program;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.DataframeGraphValueDomain;
import it.unive.pylisa.checks.DataframeDumper;
import it.unive.pylisa.checks.DataframeStructureConstructor;
import it.unive.pylisa.checks.ExceptionsCheck;
import java.io.IOException;
import java.util.Arrays;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.HelpFormatter;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.logging.log4j.Level;
import org.apache.logging.log4j.Logger;
import org.graphstream.util.parser.ParseException;

public class PyLiSA {

	private static Logger LOG = org.apache.logging.log4j.LogManager.getLogger(PyLiSA.class);

	public static void main(
			String[] args)
			throws Exception {
		// Define options
		Options options = new Options();

		Option helpOption = new Option("h", "help", false, "Print this help message");
		Option sourceOption = Option.builder("s")
				.longOpt("source")
				.hasArgs()
				.desc("Source files (e.g. -s file1 file2 file3) - the first input is deemed the main file")
				.required(false) // Will validate manually if help is not used
				.build();

		Option outdirOption = Option.builder("o")
				.longOpt("outdir")
				.hasArg()
				.desc("Output directory")
				.required(false)
				.build();

		Option logLevel = Option.builder("l")
				.longOpt("log-level")
				.hasArg()
				.desc("Log level: (INFO, DEBUG, WARNING, ERROR, FATAL, TRACE, ALL, OFF)")
				.required(false)
				.build();

		Option checker = Option.builder("c")
				.longOpt("checker")
				.hasArg()
				.desc("Checker: (Exceptions)")
				.required(false)
				.build();

		Option numericalDomainOption = Option.builder("n")
				.longOpt("numericalDomain")
				.hasArg()
				.desc("Numerical domain: (ConstantPropagation)")
				.required(false)
				.build();

		Option mode = Option.builder("m")
				.longOpt("mode")
				.hasArg()
				.desc("Execution mode: (Statistics, Debug [DEFAULT], Dataframes)")
				.required(false)
				.build();

		Option version = Option.builder("v")
				.longOpt("version")
				.desc("Version of the tool")
				.required(false)
				.build();

		Option noHtmlOutput = Option.builder()
				.longOpt("no-html")
				.desc("Disable HTML output (enabled by default)")
				.required(false)
				.build();

		Option dumpExcs = Option.builder("e")
				.longOpt("dump-exceptions")
				.desc("When in statistics mode, log exceptions to standard output")
				.required(false)
				.build();

		Option debugInfo = Option.builder("d")
				.longOpt("debug-information")
				.desc("Produce debug information for the analysis (open calls, bottom states, ...)")
				.required(false)
				.build();

		options.addOption(noHtmlOutput);

		options.addOption(helpOption);
		options.addOption(sourceOption);
		options.addOption(outdirOption);
		options.addOption(logLevel);
		options.addOption(checker);
		options.addOption(numericalDomainOption);
		options.addOption(mode);
		options.addOption(version);
		options.addOption(noHtmlOutput);
		options.addOption(dumpExcs);
		options.addOption(debugInfo);
		// Create parser and formatter
		CommandLineParser parser = new DefaultParser();
		HelpFormatter formatter = new HelpFormatter();
		String[] sources = new String[0];
		String outdir = "";
		String checkerName = "", numericalDomain = "", executionMode = "Debug";
		boolean htmlOutput = true;
		boolean dumpExceptions = false;
		boolean debug = false;

		try {
			CommandLine cmd = parser.parse(options, args);

			// Handle help
			if (cmd.hasOption("h") || args.length == 0) {
				formatter.printHelp("pylisa", options, true);
				System.exit(0);
			}

			if (cmd.hasOption("v")) {
				Package pylisaPkg = PyLiSA.class.getPackage();
				String implementationVersion = pylisaPkg.getImplementationVersion();
				System.out.println("PyLiSA version: " + implementationVersion);
				System.exit(0);
			}

			// Check required manually if help was not triggered
			if (!cmd.hasOption("s") || !cmd.hasOption("o")) {
				throw new ParseException("Missing required options: --source and/or --outdir");
			}

			// Check required manually if help was not triggered
			if (!cmd.hasOption("n") || !cmd.hasOption("o")) {
				throw new ParseException("Missing required options: --numericalDomain");
			}
			if (cmd.hasOption("l")) {
				String log4jLevelName = cmd.getOptionValue("l").toUpperCase();
				Level level = Level.getLevel(log4jLevelName);
				if (level == null) {
					throw new ParseException("Invalid log level: " + log4jLevelName);
				}
				LogManager.setLogLevel(level);
			}

			if (cmd.hasOption("no-html")) {
				htmlOutput = false;
			}

			if (cmd.hasOption("e")) {
				dumpExceptions = true;
			}

			if (cmd.hasOption("d")) {
				debug = true;
			}

			checkerName = cmd.getOptionValue("c", "Assert");
			numericalDomain = cmd.getOptionValue("n");

			sources = cmd.getOptionValues("s");
			outdir = cmd.getOptionValue("o");
			if (!outdir.endsWith("/")) {
				outdir += "/";
			}
			// Output
			LOG.info("Source files:");
			for (String file : sources) {
				LOG.info(" - " + file);
			}

			LOG.info("Output directory: " + outdir);

			if (cmd.hasOption("m"))
				executionMode = cmd.getOptionValue("m");

		} catch (ParseException e) {
			LOG.error("Error: " + e.getMessage());
			formatter.printHelp("jlisa", options, true);
			System.exit(1);
		}

		String[] copy = Arrays.copyOfRange(args, 1, args.length);
		switch (executionMode) {
		case "Debug":
			runDebug(sources, outdir, checkerName, numericalDomain, htmlOutput, debug);
			break;
		case "Statistics":
			runStatistics(sources, outdir, checkerName, numericalDomain, htmlOutput, dumpExceptions, debug);
			break;
		case "Dataframes":
			dataframes(copy);
			break;
		default:
			LOG.error("Unknown execution mode: " + executionMode);
			System.exit(1);
		}
	}

	private static void dataframes(
			String[] args)
			throws IOException {
		if (args.length < 2) {
			System.err.println("Dataframes mode needs two arguments: the file to analyze and the working directory");
			System.exit(-1);
		}

		String file = args[0];
		String workdir = args[1];
		PyFrontend translator = new PyFrontend();
		Program program = translator.parseFromListOfFile(Arrays.asList(file));

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = workdir;
		conf.outputs.add(new JSONReportDumper());
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.semanticChecks.add(new DataframeDumper());
		conf.semanticChecks.add(new DataframeStructureConstructor());

		conf.analysis = new SimpleAbstractDomain<HeapEnvWithFields, DataframeGraphDomain, TypeEnvironment<TypeSet>>(
				new FieldSensitivePointBasedHeap(), new DataframeGraphValueDomain(), new InferredTypes());

		LiSA lisa = new LiSA(conf);
		LiSAReport report = lisa.run(program);
		if (!report.getWarnings().isEmpty()) {
			System.out.println("The analysis generated the following warnings:");
			for (Message w : report.getWarnings())
				System.out.println("  " + w);
		}
	}

	private static void runDebug(
			String[] sources,
			String outdir,
			String checkerName,
			String numericalDomain,
			boolean htmlOutput,
			boolean debug)
			throws IOException,
			ParseException,
			ParsingException {
		PyFrontend frontend = runFrontend(sources);
		runAnalysis(outdir, checkerName, numericalDomain, frontend, htmlOutput, debug);
	}

	private static void runStatistics(
			String[] sources,
			String outdir,
			String checkerName,
			String numericalDomain,
			boolean htmlOutput,
			boolean dumpExceptions,
			boolean debug) {
		PyFrontend frontend = null;
		try {
			frontend = runFrontend(sources);
		} catch (Throwable e) {
			Throwable root = e;
			while (root.getCause() != null)
				root = root.getCause();
			CSVExceptionWriter.writeCSV(outdir + "frontend.csv", root);
			LOG.error("Some errors occurred in the frontend outside the parsing phase. Check " + outdir
					+ "/frontend.csv file.");
			if (dumpExceptions)
				root.printStackTrace(System.out);
			System.exit(1);
		}
		try {
			runAnalysis(outdir, checkerName, numericalDomain, frontend, htmlOutput, debug);
		} catch (Throwable e) {
			Throwable root = e;
			while (root.getCause() != null)
				root = root.getCause();
			CSVExceptionWriter.writeCSV(outdir + "analysis.csv", root);
			LOG.error("Some errors occurred during the analysis. Check " + outdir + "analysis.csv file.");
			if (dumpExceptions)
				root.printStackTrace(System.out);
			System.exit(1);
		}
	}

	private static PyFrontend runFrontend(
			String[] sources)
			throws IOException,
			ParsingException {
		PyFrontend frontend = null;
		frontend = new PyFrontend();
		frontend.parseFromListOfFile(Arrays.stream(sources).toList());
		return frontend;
	}

	private static void runAnalysis(
			String outdir,
			String checkerName,
			String numericalDomain,
			PyFrontend frontend,
			boolean htmlOutput,
			boolean debug)
			throws ParseException {
		Program p = frontend.getProgram();
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = outdir;
		conf.outputs.add(new JSONResults<>());
		conf.outputs.add(new JSONReportDumper());
		conf.interproceduralAnalysis = new InliningAnalysis<>(150, false);
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		switch (checkerName) {
		case "Assert":
			conf.semanticChecks.add(new ExceptionsCheck<>());
			break;
		case "":
			break;
		default:
			throw new ParseException("Invalid checker name: " + checkerName);
		}
		ValueDomain<?> domain;
		switch (numericalDomain) {
		case "ConstantPropagation":
			domain = new ConstantValuePropagation();
			break;
		default:
			throw new ParseException("Invalid numerical domain name: " + numericalDomain);
		}

		conf.analysis = new Reachability<>(new SimpleAbstractDomain<>(
				new FieldSensitivePointBasedHeap(),
				domain,
				new InferredTypes()));

		if (htmlOutput)
			conf.outputs.add(new HtmlResults<>(true));

		if (debug) {
			conf.asynchronousListeners.add(new BottomTopListener(false));
			conf.asynchronousListeners.add(new CallResolutionListener());
		}

		LiSA lisa = new LiSA(conf);
		lisa.run(p);
	}
}
