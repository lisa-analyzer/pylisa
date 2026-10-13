package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NarrowingTest extends AnalysisTestExecutor {

	@Test
	public void testNarrowing() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "narrowing";
		conf.programFile = "narrowing.py";
		perform(conf);
	}
}
