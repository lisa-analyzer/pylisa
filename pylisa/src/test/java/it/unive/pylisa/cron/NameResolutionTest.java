package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NameResolutionTest extends AnalysisTestExecutor {

	@Test
	public void testNameResolution() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "name-resolution";
		conf.programFile = "name-resolution.py";
		perform(conf);
	}
}
