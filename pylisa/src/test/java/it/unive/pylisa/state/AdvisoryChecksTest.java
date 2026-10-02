package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.checks.Advisory;
import it.unive.pylisa.checks.CallableNotCalledCheck;
import it.unive.pylisa.checks.StoredNoneResultCheck;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks the two no-effect advisories: a statement that only names a callable,
 * and a stored result that is always {@code None}. Each fires exactly on the
 * lines labelled in the positive programs, and never in the negative ones.
 */
class AdvisoryChecksTest {

	private static final String PROGRAMS = "src/test/resources/programs/advisories/";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void callableNamedButNotCalled(
			AnalysisConfig config)
			throws Exception {
		CallableNotCalledCheck<?, ?> check = new CallableNotCalledCheck<>();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "callable_not_called_positive.py", config,
				List.of(check));
		assertEquals(lines(helper, "@function", "@method", "@bound", "@class", "@native"),
				lines(check.advisories()));
		check.advisories().forEach(advisory -> assertEquals(Advisory.Kind.CALLABLE_NOT_CALLED, advisory.kind()));
		assertTrue(check.advisories().stream()
				.anyMatch(advisory -> advisory.message().equals("helper is a function and is not called")),
				check.advisories().toString());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void noAdvisoryForCallsAssignmentsConditionsOrValuesThatMayNotBeCallable(
			AnalysisConfig config)
			throws Exception {
		CallableNotCalledCheck<?, ?> check = new CallableNotCalledCheck<>();
		StateTestHelper.analyse(PROGRAMS + "callable_not_called_negative.py", config, List.of(check));
		assertEquals(List.of(), check.advisories());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void storedResultThatIsAlwaysNone(
			AnalysisConfig config)
			throws Exception {
		StoredNoneResultCheck<?, ?> check = new StoredNoneResultCheck<>();
		StateTestHelper helper = StateTestHelper.analyse(PROGRAMS + "stored_none_positive.py", config,
				List.of(check));
		assertEquals(lines(helper, "@native", "@function"), lines(check.advisories()));
		check.advisories().forEach(advisory -> assertEquals(Advisory.Kind.STORED_NONE_RESULT, advisory.kind()));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void noAdvisoryForResultsSometimesNoneGeneratorsOrNeverReached(
			AnalysisConfig config)
			throws Exception {
		StoredNoneResultCheck<?, ?> check = new StoredNoneResultCheck<>();
		StateTestHelper.analyse(PROGRAMS + "stored_none_negative.py", config, List.of(check));
		assertEquals(List.of(), check.advisories());
	}

	private static List<Integer> lines(
			StateTestHelper helper,
			String... labels) {
		return Stream.of(labels).map(helper::lineOf).sorted().toList();
	}

	/**
	 * Yields the lines of the advisories, one per advisory, so that two
	 * advisories on one line are told apart.
	 */
	private static List<Integer> lines(
			List<Advisory> advisories) {
		return advisories.stream().map(advisory -> ((SourceCodeLocation) advisory.location()).getLine()).sorted()
				.toList();
	}
}
