package it.unive.pylisa.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.analysis.Val;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks how arguments that match the parameters of a callee, or do not, are
 * passed: a mismatch gives a native callee an unknown value for each of its
 * parameters, and keyword arguments of a constructor reach its keyword-only
 * parameters.
 */
class ArgumentMismatchTest {

	private static final String PROGRAM = "src/test/resources/programs/calls/native_argument_mismatch.py";

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void nativeGetsAnUnknownValuePerParameter(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper.analyse(PROGRAM, config);
		assertEquals(Val.exact("a"), helper.after("@matched").value("matched"));
		assertEquals(Val.top(), helper.after("@mismatched").value("mismatched"));
	}

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void constructorKeywordArgumentsReachKeywordOnlyParameters(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper helper = StateTestHelper
				.analyse("src/test/resources/programs/calls/keyword_only_constructor.py", config);
		// the arguments are passed to object.__new__ too, which accepts any
		assertTrue(helper.after("@created").object("p").type().startsWith("__main__.Point"),
				helper.after("@created").object("p").type());
		assertEquals(Val.exact(0), helper.after("@created").object("p").field("x"));
		assertEquals(Val.exact(5), helper.after("@created").object("p").field("y"));
	}
}
