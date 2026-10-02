package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.testnatives.CarryNative;
import it.unive.pylisa.program.ProgramSettings;
import it.unive.pylisa.libraries.natives.CarriedMarks;
import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import it.unive.pylisa.testnatives.MarkedOnlyNative;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks how the errors a model raises record the assumptions their branch
 * depends on: a mark stays with the executions that need it, and never
 * reaches executions that do not.
 */
class AssumptionMarksTest {

	private static final String PROGRAM = "src/test/resources/programs/marks/marks.py";

	private static final String VALUE_ERROR = "builtins.ValueError";

	private static final Set<String> MARKED = Set.of(MarkedOnlyNative.ASSUMPTION);

	private static Set<Set<String>> marks(
			StateTestHelper helper,
			String label) {
		return helper.after(label).errorMarks(VALUE_ERROR, helper.lineOf(label));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void markedAndUnmarkedErrorsOfOneCallStayApart(
			AnalysisConfig config)
			throws Exception {
		assertEquals(Set.of(Set.of(), MARKED), marks(StateTestHelper.analyse(PROGRAM, config), "@both"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aJoinWithUnmarkedExecutionsDropsTheMarkWhateverTheOrder(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Set.of(Set.of()), marks(helper, "@marked_first"));
		assertEquals(Set.of(Set.of()), marks(helper, "@plain_first"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aJoinWithNoExecutionKeepsTheMark(
			AnalysisConfig config)
			throws Exception {
		assertEquals(Set.of(MARKED), marks(StateTestHelper.analyse(PROGRAM, config), "@with_unreachable"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void markedErrorReachesTheCallerAsAnErrorOfTheProgram(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Set.of(MARKED), marks(helper, "@inside"));
		assertEquals(Set.of(Set.of()), marks(helper, "@caller"));
		assertTrue(helper.after("@inside").everyErrorRaisedWithinProgram(), helper.after("@inside").toString());
		assertTrue(helper.after("@caller").everyErrorRaisedWithinProgram(), helper.after("@caller").toString());
	}

	private static final ProgramSettings CARRYING = ProgramSettings.NONE.with(CarriedMarks.class,
			new CarriedMarks(Set.of(CarryNative.CARRIED)));

	private static final Set<String> CARRIED = Set.of(CarryNative.CARRIED);

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aCarriedMarkAppliesToLaterCallsOnlyWhereItIsSetOnEveryExecution(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse("src/test/resources/programs/marks/carried.py", config,
				List.of(), CARRYING);
		assertEquals(Set.of(Set.of()), marks(helper, "@before"));
		// set on some executions only: joined with unset, so not carried
		assertEquals(Set.of(Set.of()), marks(helper, "@some_paths"));
		assertEquals(Set.of(CARRIED), marks(helper, "@after"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void aCarriedMarkSetInACalleeAppliesInTheCaller(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse("src/test/resources/programs/marks/carried_in_callee.py",
				config, List.of(), CARRYING);
		assertEquals(Set.of(CARRIED), marks(helper, "@after_callee"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void carryingAMarkTheProgramDoesNotCarryFailsLoudly(
			AnalysisConfig config) {
		// its flag would not be unset at the entry, so a join could read it set
		assertThrows(Exception.class,
				() -> StateTestHelper.analyse("src/test/resources/programs/marks/carried.py", config));
	}
}
