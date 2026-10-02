package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.testing.Point;
import it.unive.pylisa.testing.StateTestHelper;
import java.util.Set;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.opentest4j.AssertionFailedError;

/**
 * Checks that {@link StateTestHelper} reads variables, heap objects and their
 * fields correctly on a plain Python program, before it is used to check ROS 2
 * programs.
 */
class StateTestHelperSmokeTest {

	private static final String PROGRAM = "src/test/resources/programs/helper/smoke.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void readsModuleVariables(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.exact("a"), helper.after("@x").value("x"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void readsFieldsOfObjects(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		Point point = helper.after("@f");
		assertEquals(Val.exact("b"), point.value("o.f"));
		assertEquals(Val.exact("b"), point.object("o").field("f"));
		assertTrue(point.object("o").type().endsWith("C@1:0"), point.object("o").type());
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void readsLocalVariablesOfFunctions(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.exact("ns/"), helper.after("@q").value("q"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void distinguishesObjectsByAllocationSite(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		Point point = helper.after("@two");
		assertNotEquals(point.object("o"), point.object("o2"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void joinsTheValuesOfBothBranches(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		Val expected = config == AnalysisConfig.CP ? Val.top() : Val.oneOf(Set.of("a", "b"));
		assertEquals(expected, helper.after("@branch").value("u"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void rejectsLinesWithoutStatementsAndMissingData(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertThrows(AssertionFailedError.class, () -> helper.after(3));
		assertThrows(AssertionFailedError.class, () -> helper.after("@x").value("no_such_variable"));
		assertThrows(AssertionFailedError.class, () -> helper.after("@missing"));
	}
}
