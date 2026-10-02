package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.checks.AssertionVerdict;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.opentest4j.AssertionFailedError;

/**
 * Checks the translation of {@code assert} statements and the verdicts given
 * to them, on a plain Python program.
 */
class AssertCheckerTest {

	private static final String PROGRAM = "src/test/resources/programs/assertions/verdicts.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void givesEachAssertionItsVerdict(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(AssertionVerdict.PROVED, helper.verdict("@proved"));
		assertEquals(AssertionVerdict.PROVED, helper.verdict("@proved_ne"));
		assertEquals(AssertionVerdict.PROVED, helper.verdict("@proved_message"));
		assertEquals(AssertionVerdict.UNREACHABLE, helper.verdict("@unreachable"));
		assertEquals(AssertionVerdict.FAILS, helper.verdict("@fails"));
		assertEquals(AssertionVerdict.UNREACHABLE, helper.verdict("@after_failure"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aFailingAssertionStopsTheExecutionsThatViolateIt(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertTrue(helper.after("@proved").isReachable());
		assertFalse(helper.after("@fails").isReachable());
		assertTrue(helper.after("@fails").errors().contains("builtins.AssertionError"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anUndecidedConditionMayFail(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse("src/test/resources/programs/assertions/undecided.py", config);
		assertEquals(AssertionVerdict.MAY_FAIL, helper.verdict("@may"));
		assertTrue(helper.after("@may").isReachable());
		assertTrue(helper.after("@may").errors().contains("builtins.AssertionError"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void allProvedFailsOnAnyOtherVerdict(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertThrows(AssertionFailedError.class, helper::assertAllProved);
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void anAssertionInAFunctionNoAnalysedCodeRunsIsNotAnalysed(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse("src/test/resources/programs/assertions/not_analysed.py",
				config);
		assertEquals(AssertionVerdict.NOT_ANALYSED, helper.verdict("@never"));
		assertEquals(AssertionVerdict.PROVED, helper.verdict("@called"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void noVerdictIsDefiniteWhenPartOfTheProgramIsTranslatedUnsoundly(
			AnalysisConfig config)
			throws Exception {
		// the except handler is not analysed, so ok might be False
		StateTestHelper helper = StateTestHelper
				.analyse("src/test/resources/programs/assertions/unsound_translation.py", config);
		assertEquals(AssertionVerdict.MAY_FAIL, helper.verdict("@after_try"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aReachedConditionThatCannotBeEvaluatedDecidesNothing(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper
				.analyse("src/test/resources/programs/assertions/undecided_condition.py", config);
		assertTrue(helper.verdict("@plural") != AssertionVerdict.UNREACHABLE, helper.asserts().toString());
		assertTrue(helper.verdict("@bool_count") != AssertionVerdict.UNREACHABLE, helper.asserts().toString());
	}
}
