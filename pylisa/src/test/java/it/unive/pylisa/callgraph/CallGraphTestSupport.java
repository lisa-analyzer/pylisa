package it.unive.pylisa.callgraph;

import it.unive.lisa.LiSA;
import it.unive.lisa.ReportingTool;
import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.events.Event;
import it.unive.lisa.events.EventListener;
import it.unive.lisa.interprocedural.callgraph.CallGraph;
import it.unive.lisa.interprocedural.callgraph.CallGraphNode;
import it.unive.lisa.interprocedural.callgraph.events.CallResolved;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.util.file.FileManager;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.PythonAnalysis;
import it.unive.pylisa.program.ProgramSettings;
import it.unive.pylisa.testing.TestDirectories;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

/**
 * Runs a Python program through pylisa's analysis and keeps what the call
 * graph recorded: the call graph itself, and every resolution event it
 * posted, in posting order.
 */
final class CallGraphTestSupport {

	private static final String RESULTS = "callgraph-results";

	private CallGraphTestSupport() {
	}

	/**
	 * What one analysis of a program recorded.
	 *
	 * @param callGraph the call graph after the analysis
	 * @param events    the resolution events, in the order they were posted
	 * @param lines     the lines of the program, to find labelled calls
	 */
	record Run(CallGraph callGraph, List<CallResolved> events, List<String> lines) {

		/**
		 * Yields the call sites of the code member with the given full name,
		 * such as {@code testnatives.echo::$call}.
		 *
		 * @param fullName the full name of the member
		 *
		 * @return the call sites; empty if the member is not in the call
		 *             graph
		 */
		Collection<Call> sitesOf(
				String fullName) {
			for (CallGraphNode node : callGraph.getNodes()) {
				CodeMember member = node.getCodeMember();
				if (member.getDescriptor().getFullName().equals(fullName))
					return callGraph.getCallSites(member);
			}
			return List.of();
		}

		/**
		 * Yields the call sites of the code members whose full name ends
		 * with the given suffix, such as {@code .greet::$call}: the full
		 * names of Python functions include the location of their class.
		 *
		 * @param suffix the end of the full names
		 *
		 * @return the call sites
		 */
		Collection<Call> sitesEndingWith(
				String suffix) {
			List<Call> sites = new ArrayList<>();
			for (CallGraphNode node : callGraph.getNodes())
				if (node.getCodeMember().getDescriptor().getFullName().endsWith(suffix))
					sites.addAll(callGraph.getCallSites(node.getCodeMember()));
			return sites;
		}

		/**
		 * Yields the call sites of the method with the given name of the
		 * class with the given name, whose full name carries the location of
		 * the class, as in {@code __main__.Sub@10:0.__init__::$call}.
		 *
		 * @param className the name of the class
		 * @param method    the name of the method
		 *
		 * @return the call sites
		 */
		Collection<Call> sitesOfMethod(
				String className,
				String method) {
			List<Call> sites = new ArrayList<>();
			for (CallGraphNode node : callGraph.getNodes()) {
				String name = node.getCodeMember().getDescriptor().getFullName();
				if (name.contains("." + className + "@") && name.endsWith("." + method + "::$call"))
					sites.addAll(callGraph.getCallSites(node.getCodeMember()));
			}
			return sites;
		}

		/**
		 * Yields every call site the call graph lists, for any member.
		 *
		 * @return the call sites
		 */
		Collection<Call> allSites() {
			List<Call> sites = new ArrayList<>();
			for (CallGraphNode node : callGraph.getNodes())
				sites.addAll(callGraph.getCallSites(node.getCodeMember()));
			return sites;
		}

		/**
		 * Yields the resolution events of the calls on one line.
		 *
		 * @param line the line, starting from 1
		 *
		 * @return the events, in posting order
		 */
		List<CallResolved> eventsAt(
				int line) {
			return events.stream()
					.filter(event -> event.getOriginal().getLocation() instanceof SourceCodeLocation location
							&& location.getLine() == line)
					.toList();
		}

		/**
		 * Yields the line marked with a label, written as a comment
		 * {@code # @label} at the end of the line.
		 *
		 * @param label the label, starting with {@code @}
		 *
		 * @return the line, starting from 1
		 */
		int lineOf(
				String label) {
			return CallGraphTestSupport.lineOf(lines, label);
		}
	}

	/**
	 * Yields the line of a program marked with a label, written as a comment
	 * {@code # @label} at the end of the line.
	 *
	 * @param lines the lines of the program
	 * @param label the label, starting with {@code @}
	 *
	 * @return the line, starting from 1
	 */
	static int lineOf(
			List<String> lines,
			String label) {
		for (int i = 0; i < lines.size(); i++)
			if (lines.get(i).matches(".*#\\s*" + label + "\\b.*"))
				return i + 1;
		throw new AssertionError("No line is marked " + label);
	}

	/**
	 * Analyses a program with the configuration pylisa installs, recording
	 * the call graph and its resolution events.
	 *
	 * @param program the path of the Python entry file, relative to the
	 *                    project directory
	 * @param config  the value configuration
	 *
	 * @return what the analysis recorded
	 *
	 * @throws Exception if the program cannot be read or analysed
	 */
	static Run analyse(
			String program,
			AnalysisConfig config)
			throws Exception {
		return analyse(program, config, null);
	}

	/**
	 * Analyses a program with the configuration pylisa installs, but with
	 * the given call graph, recording the call graph and its resolution
	 * events.
	 *
	 * @param program   the path of the Python entry file, relative to the
	 *                      project directory
	 * @param config    the value configuration
	 * @param callGraph the call graph to use, or {@code null} for the one
	 *                      the configuration installs
	 *
	 * @return what the analysis recorded
	 *
	 * @throws Exception if the program cannot be read or analysed
	 */
	static Run analyse(
			String program,
			AnalysisConfig config,
			CallGraph callGraph)
			throws Exception {
		Path path = Path.of(program);
		String stem = path.getFileName().toString().replaceFirst("\\.py$", "");
		String workdir = TestDirectories.of(RESULTS).resolve(stem).resolve(config.name()).toString();
		FileManager.forceDeleteFolder(workdir);
		LiSAConfiguration conf = new LiSAConfiguration();
		conf.workdir = workdir;
		config.configure(conf);
		if (callGraph != null)
			conf.callGraph = callGraph;
		Recorder recorder = new Recorder();
		conf.synchronousListeners.add(recorder);
		Grab<?, ?> grab = new Grab<>();
		conf.semanticChecks.add(grab);
		new LiSA(conf).run(PythonAnalysis.translate(program, List.of(), ProgramSettings.NONE));
		if (grab.callGraph == null)
			throw new AssertionError("The analysis of " + program + " reached no code");
		return new Run(grab.callGraph, List.copyOf(recorder.events),
				Files.readAllLines(path, StandardCharsets.UTF_8));
	}

	/** Records the resolution events, in posting order. */
	private static final class Recorder implements EventListener {

		private final List<CallResolved> events = Collections.synchronizedList(new ArrayList<>());

		@Override
		public void onEvent(
				Event event,
				ReportingTool tool) {
			if (event instanceof CallResolved resolved)
				events.add(resolved);
		}
	}

	/** Keeps the call graph of the run. */
	private static final class Grab<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
			implements
			SemanticCheck<A, D> {

		private CallGraph callGraph;

		@Override
		public boolean visit(
				SemanticTool<A, D> tool,
				CFG graph) {
			callGraph = tool.getCallGraph();
			return true;
		}
	}
}
