package it.unive.pylisa.checks;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.program.PySourceCodeLocation;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks which statements the always-raises check reports: a call that
 * raises in every context where it is reached, and no call that completes in
 * some context or that may block instead of returning.
 */
class AlwaysRaisesCheckTest {

	private static final String PROGRAMS = "src/test/resources/programs/checks/always_raises/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anArgumentBoundToTheWrongParameterMakesTheCallAlwaysRaise(
			AnalysisConfig config)
			throws Exception {
		Run run = run("wrong_binding.py", config);
		assertEquals(Map.of(run.helper.lineOf("@unpack"), List.of("builtins.TypeError")), run.findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void theCallWithItsArgumentsBoundRightIsNotReported(
			AnalysisConfig config)
			throws Exception {
		assertEquals(Map.of(), run("right_binding.py", config).findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aStatementThatCompletesInSomeContextIsNotReported(
			AnalysisConfig config)
			throws Exception {
		Run run = run("contexts.py", config);
		// the call in the helper raises when reached through one caller only
		assertEquals(Map.of(run.helper.lineOf("@always"), List.of("builtins.ValueError"),
				run.helper.lineOf("@through_helper"), List.of("builtins.ValueError")), run.findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aCallThatMayBlockInsteadOfReturningIsNotReported(
			AnalysisConfig config)
			throws Exception {
		Run run = run("blocking.py", config);
		assertEquals(Map.of(run.helper.lineOf("@fails"), List.of("builtins.ValueError")), run.findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aFunctionUsedAsAValueIsNotJudgedFromTheCallsTheAnalysisSees(
			AnalysisConfig config)
			throws Exception {
		Run run = run("escaping.py", config);
		// the function is also called by code the analysis does not run
		assertEquals(Map.of(run.helper.lineOf("@direct"), List.of("builtins.ValueError")), run.findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aFunctionHandedOnByALambdaOrAnUnreachedFunctionIsNotJudged(
			AnalysisConfig config)
			throws Exception {
		Run run = run("escaping_lambda.py", config);
		// validate escapes through a lambda, check through setup and deep
		// through hook, which no analysed code calls: inside them nothing is
		// reported, while the calls of the program itself still are
		assertEquals(Map.of(run.helper.lineOf("@validate"), List.of("builtins.ValueError"),
				run.helper.lineOf("@check"), List.of("builtins.ValueError"),
				run.helper.lineOf("@deep"), List.of("builtins.ValueError")), run.findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aMethodAnEscapingFunctionCallsOnlyUnseenIsNotJudged(
			AnalysisConfig config)
			throws Exception {
		Run run = run("escaping_dispatch.py", config);
		// the holder of the callback may call it with a sink, so write runs
		// with other arguments than the one the analysis sees
		assertEquals(Map.of(run.helper.lineOf("@direct"), List.of("builtins.ValueError")), run.findings);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anUnsoundTranslationMakesTheRunBestEffort(
			AnalysisConfig config)
			throws Exception {
		AlwaysRaisesCheck<?, ?> raises = new AlwaysRaisesCheck<>();
		StoredNoneResultCheck<?, ?> storedNone = new StoredNoneResultCheck<>();
		CallableNotCalledCheck<?, ?> notCalled = new CallableNotCalledCheck<>();
		StateTestHelper.analyse(PROGRAMS + "try_bypass.py", config, List.of(raises, storedNone, notCalled));
		// the handler is bypassed, so the caller of the function with the try
		// sees None only, while Python returns the object
		assertTrue(raises.sawUnsoundTranslation());
		assertTrue(storedNone.sawUnsoundTranslation());
		assertTrue(notCalled.sawUnsoundTranslation());
	}

	private record Run(StateTestHelper helper, Map<Integer, List<String>> findings) {
	}

	private static Run run(
			String program,
			AnalysisConfig config)
			throws Exception {
		AlwaysRaisesCheck<?, ?> check = new AlwaysRaisesCheck<>();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + program, config, List.of(check));
		Map<Integer, List<String>> findings = new TreeMap<>();
		for (AlwaysRaisesCheck.Finding finding : check.getFindings())
			findings.put(((PySourceCodeLocation) finding.statement().getLocation()).getStartLine(),
					List.copyOf(finding.exceptions()));
		return new Run(helper, findings);
	}
}
