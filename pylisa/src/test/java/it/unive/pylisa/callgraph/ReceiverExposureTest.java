package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNull;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.InstrumentedReceiverRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.PyCallGraph;
import it.unive.pylisa.callgraph.CallGraphTestSupport.Run;
import java.util.Collection;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks which operand of a call the call graph binds to the receiver of a target, for readers
 * that read the object a method is called on: the receiver a method call passes, the object an
 * instantiation creates, and none for a function reached through a module or a method called
 * through its class.
 */
class ReceiverExposureTest {

	private static final String PROGRAM = "src/test/resources/callgraph/receivers/receivers.py";

	private static final String TWO_CLASSES = "src/test/resources/callgraph/receivers/two_classes.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void methodCalledOnAnObjectExposesTheObject(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(PROGRAM, config);
		assertEquals("a", receiver(run, run.sitesOfMethod("A", "m"), "@method").toString().replace("__main__::", ""));
		assertEquals("p", receiver(run, run.sitesOf("testnatives.Plain.echo::$call"), "@native_method").toString()
				.replace("__main__::", ""));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void instantiationExposesTheObjectItCreates(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(PROGRAM, config);
		assertInstanceOf(InstrumentedReceiverRef.class,
				receiver(run, run.sitesOf("testnatives.Plain.__init__::$call"), "@construct"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void superCallExposesTheObjectBeingInitialised(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(PROGRAM, config);
		assertEquals("self", receiver(run, run.sitesOf("testnatives.Plain.__init__::$call"), "@super").toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void functionThroughAModuleOrMethodThroughItsClassExposesNothing(
			AnalysisConfig config)
			throws Exception {
		Run run = CallGraphTestSupport.analyse(PROGRAM, config);
		assertNull(receiver(run, run.sitesOf("testnatives.echo::$call"), "@module"));
		assertNull(receiver(run, run.sitesOfMethod("A", "m"), "@unbound"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void receiverThatIsAClassInSomeContextExposesNothing(
			AnalysisConfig config)
			throws Exception {
		// r.m(v) is reached with an object and with the class: the class
		// would be read as the object the method is called on
		Run run = CallGraphTestSupport.analyse(PROGRAM, config);
		assertNull(receiver(run, run.sitesOfMethod("A", "m"), "@mixed"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void instantiationOfOneOfSeveralClassesExposesNothing(
			AnalysisConfig config)
			throws Exception {
		// both classes reach the same __init__ through equal nested calls,
		// whose receiver operands share one stored state: the object of one
		// class would be read as the object of the other
		Run run = CallGraphTestSupport.analyse(TWO_CLASSES, config);
		Collection<Call> sites = run.sitesOf("testnatives.Plain.__init__::$call");
		assertNull(receiver(run, sites, "@two_classes"));
		assertInstanceOf(InstrumentedReceiverRef.class, receiver(run, sites, "@one_class"));
	}

	/**
	 * Yields the operand bound to the receiver of the target at the call site on the labelled line.
	 */
	private static Expression receiver(
			Run run,
			Collection<Call> sites,
			String label) {
		int line = run.lineOf(label);
		Call site = sites.stream()
				.filter(call -> ((SourceCodeLocation) call.getLocation()).getLine() == line)
				.findFirst()
				.orElseThrow(() -> new AssertionError("no site on line " + line + " among " + sites));
		PyCallGraph graph = (PyCallGraph) run.callGraph();
		CodeMember target = run.callGraph().getNodes().stream()
				.map(node -> node.getCodeMember())
				.filter(member -> run.callGraph().getCallSites(member).contains(site))
				.findFirst()
				.orElseThrow();
		return graph.receiverOf(site, target);
	}
}
