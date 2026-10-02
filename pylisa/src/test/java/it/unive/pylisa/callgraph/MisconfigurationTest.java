package it.unive.pylisa.callgraph;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.callgraph.events.CallResolved;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.callgraph.CallGraphTestSupport.Run;
import it.unive.pylisa.cfg.statement.PyInstantiation;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.regex.Pattern;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that an analysis whose call graph does not dispatch Python calls
 * stops at the first one, saying where, and that the check accepts a
 * resolution shared by equal calls.
 */
class MisconfigurationTest {

	private static final String DIR = "src/test/resources/callgraph/misconfiguration/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anotherCallGraphFailsAtTheFirstPythonCall(
			AnalysisConfig config)
			throws Exception {
		int line = CallGraphTestSupport.lineOf(Files.readAllLines(Path.of(DIR + "one_call.py")), "@call");
		Exception error = assertThrows(Exception.class,
				() -> CallGraphTestSupport.analyse(DIR + "one_call.py", config, new RTACallGraph()));
		String messages = messages(error);
		assertTrue(messages.contains("Python calls need pylisa's call graph"), messages);
		assertTrue(Pattern.compile("one_call\\.py'?:" + line + ":").matcher(messages).find(), messages);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void equalNestedCallSharesTheResolutionOfTheFirst(
			AnalysisConfig config)
			throws Exception {
		// k(v) is resolved twice, with the callee C and with C or g, and each
		// resolution builds its own instantiation of C: both resolve their
		// call of __new__, but their calls of __init__ are equal and get the
		// same types, so the second gets the resolution of the first, which
		// the check must accept
		Run run = CallGraphTestSupport.analyse(DIR + "equal_nested_calls.py", config);
		List<CallResolved> events = run.eventsAt(run.lineOf("@make"));
		assertEquals(2, events.stream().filter(event -> !nested(event)).count(), events::toString);
		assertEquals(2, nestedCallsOf(events, ".__new__"), events::toString);
		assertEquals(1, nestedCallsOf(events, ".__init__"), events::toString);
	}

	/**
	 * Counts the resolutions of the calls an instantiation makes to the
	 * method with the given suffix.
	 */
	private static long nestedCallsOf(
			List<CallResolved> events,
			String method) {
		return events.stream().filter(MisconfigurationTest::nested)
				.filter(event -> event.getTypes()[0].stream().anyMatch(type -> type.toString().endsWith(method)))
				.count();
	}

	private static boolean nested(
			CallResolved event) {
		return event.getOriginal().getParentStatement() instanceof PyInstantiation;
	}

	private static String messages(
			Throwable error) {
		StringBuilder messages = new StringBuilder();
		for (Throwable cause = error; cause != null; cause = cause.getCause())
			messages.append(cause.getMessage()).append('\n');
		return messages.toString();
	}
}
