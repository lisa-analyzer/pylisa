package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SvVerifierTest extends AnalysisTestExecutor {

	@Test
	public void testSvVerifier() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "sv-verifier";
		conf.programFile = "sv-verifier.py";
		perform(conf);
	}
}
