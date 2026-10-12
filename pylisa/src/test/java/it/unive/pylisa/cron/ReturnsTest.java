package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ReturnsTest extends AnalysisTestExecutor {

	@Test
	public void testReturns() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "returns";
		conf.programFile = "returns.py";
		perform(conf);
	}
}
