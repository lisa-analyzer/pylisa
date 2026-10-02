package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that a model of a library callable never ends an execution silently:
 * a model that leaves no continuation for a reachable input, without
 * declaring that the callable never returns, makes the analysis fail, as does
 * a model that iterates over no values in a reachable state.
 */
class DivergenceCheckTest {

	private static final String PROGRAMS = "src/test/resources/programs/divergence/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void modelWithoutContinuationFailsTheAnalysis(
			AnalysisConfig config) {
		Throwable failure = assertThrows(Exception.class,
				() -> StateTestHelper.analyse(PROGRAMS + "silent.py", config));
		assertTrue(messages(failure).contains("testnatives.silent"), messages(failure));
		assertTrue(messages(failure).contains("no continuation"), messages(failure));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void modelWithoutContinuationFailsEvenAfterAnEarlierError(
			AnalysisConfig config) {
		Throwable failure = assertThrows(Exception.class,
				() -> StateTestHelper.analyse(PROGRAMS + "silent_after_error.py", config));
		assertTrue(messages(failure).contains("no continuation"), messages(failure));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void modelThatDeclaresItNeverReturnsIsAccepted(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "stuck.py", config);
		assertFalse(helper.after("@stuck").isReachable());
		assertFalse(helper.after("@after").isReachable(), "the rest of the program is cut");
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void modelIteratingOverNoValuesFailsTheAnalysis(
			AnalysisConfig config) {
		Throwable failure = assertThrows(Exception.class,
				() -> StateTestHelper.analyse(PROGRAMS + "empty_values.py", config));
		assertTrue(messages(failure).contains("no values"), messages(failure));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void earlierErrorsSurviveACallThatNeverReturns(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "stuck_after_error.py", config);
		assertTrue(helper.after("@stuck").errors().contains("builtins.ValueError"),
				helper.after("@stuck").errorSites().toString());
	}

	private static String messages(
			Throwable failure) {
		StringBuilder all = new StringBuilder();
		for (Throwable current = failure; current != null; current = current.getCause())
			all.append(current.getMessage()).append(" | ");
		return all.toString();
	}
}
