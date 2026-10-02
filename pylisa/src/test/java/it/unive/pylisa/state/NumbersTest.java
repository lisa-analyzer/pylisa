package it.unive.pylisa.state;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks that arithmetic and comparisons on numbers give Python's results:
 * operators group to the left, booleans are numbers, floats are doubles, and
 * chained comparisons check every comparison.
 */
class NumbersTest {

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void arithmeticFollowsPython(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper.analyse("src/test/resources/programs/python/arithmetic.py", config).assertAllProved();
	}
}
