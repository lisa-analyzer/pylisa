package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.checks.AssertionVerdict;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks a call whose arguments cannot be bound to the callee's parameters:
 * Python raises {@code TypeError} at the call, before the callee's body runs.
 * The analysis cannot always tell such a call from one whose receiver it
 * passed wrongly itself, so it lets the call raise {@code TypeError} and also
 * analyses the body with unknown arguments; it skips the body only once it
 * knows the mismatch is certain.
 */
class MissingArgumentTest {

	private static final String PROGRAM = "src/test/resources/programs/assertions/missing_argument.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theCallMayRaiseTypeError(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertTrue(helper.after("@call").errors().contains("builtins.TypeError"));
	}

	// known to fail until the analysis stops the execution at a call Python
	// certainly rejects, which needs exceptions modelled in pylisa
	@Tag("known-failing")
	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theCalleeBodyIsNotRun(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(AssertionVerdict.UNREACHABLE, helper.verdict("@inside"));
	}

	// known to fail until the analysis stops the execution at a call Python
	// certainly rejects, which needs exceptions modelled in pylisa
	@Tag("known-failing")
	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theExecutionStopsAtTheCall(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertFalse(helper.after("@call").isReachable());
		assertEquals(AssertionVerdict.UNREACHABLE, helper.verdict("@after"));
	}
}
