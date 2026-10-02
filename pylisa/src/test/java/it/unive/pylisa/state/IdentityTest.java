package it.unive.pylisa.state;

import it.unive.pylisa.analysis.AnalysisConfig;
import it.unive.pylisa.testing.StateTestHelper;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Checks the comparison {@code x is None}, which the analysis decides from the
 * value of {@code x}: nothing but {@code None} is {@code None}.
 */
class IdentityTest {

	@ParameterizedTest
	@EnumSource(AnalysisConfig.class)
	void identityWithNoneIsDecided(
			AnalysisConfig config)
			throws Exception {
		StateTestHelper.analyse("src/test/resources/programs/python/identity.py", config).assertAllProved();
	}
}
