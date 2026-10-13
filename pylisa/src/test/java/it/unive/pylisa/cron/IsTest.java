package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class IsTest extends AnalysisTestExecutor {

	@Test
	public void testIs() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "is";
		conf.programFile = "is.py";
		perform(conf);
	}
}
