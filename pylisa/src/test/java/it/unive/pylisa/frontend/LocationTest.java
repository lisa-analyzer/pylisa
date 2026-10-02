package it.unive.pylisa.frontend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.program.PySourceCodeLocation;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Checks that the location of a call gives where the whole call starts, as
 * CPython reports it, so that a report can point at it, while the location
 * still tells calls apart.
 */
class LocationTest {

	private static final String PROGRAMS = "src/test/resources/programs/python/";

	@Test
	void callWithArgumentsOnTwoLinesStartsAtItsCallee() throws Exception {
		PySourceCodeLocation location = callsOnLine("multiline_call.py", 5).get(0);
		assertEquals(5, location.getStartLine());
		assertEquals(4, location.getStartCol(), "the call starts at f, column 4 (0-based)");
	}

	@Test
	void callWhoseReceiverIsOnAnEarlierLineStartsAtTheReceiver() throws Exception {
		PySourceCodeLocation location = callsOnLine("call_chains.py", 11).get(0);
		assertEquals(10, location.getStartLine(), "the call starts at x, on the line before .f()");
		assertEquals(9, location.getStartCol());
	}

	@Test
	void chainedCallsStartTogetherButKeepDistinctLocations() throws Exception {
		List<PySourceCodeLocation> calls = callsOnLine("call_chains.py", 12);
		assertEquals(2, calls.size(), calls.toString());
		assertEquals(calls.get(0).getStartCol(), calls.get(1).getStartCol());
		assertNotEquals(calls.get(0), calls.get(1));
	}

	private static List<PySourceCodeLocation> callsOnLine(
			String program,
			int line)
			throws Exception {
		Program translated = new PyFrontend(PROGRAMS + program, false).toLiSAProgram(true);
		List<PyCall> calls = new ArrayList<>();
		for (CodeMember member : translated.getCodeMembersRecursively())
			if (member instanceof CFG cfg)
				for (Statement node : cfg.getNodes())
					collect(node, calls);
		return calls.stream()
				.map(Statement::getLocation)
				.filter(PySourceCodeLocation.class::isInstance)
				.map(PySourceCodeLocation.class::cast)
				.filter(location -> location.getLine() == line)
				.distinct()
				.sorted(Comparator.comparingInt(PySourceCodeLocation::getCol))
				.toList();
	}

	private static void collect(
			Statement statement,
			List<PyCall> calls) {
		if (statement instanceof PyCall call)
			calls.add(call);
		if (statement instanceof NaryExpression nary)
			for (Expression sub : nary.getSubExpressions())
				collect(sub, calls);
	}
}
