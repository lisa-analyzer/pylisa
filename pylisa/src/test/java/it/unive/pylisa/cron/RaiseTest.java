package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class RaiseTest extends AnalysisTestExecutor {

	@Test
	public void testRaise() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "raise";
		conf.programFile = "raise.py";
		perform(conf);
	}
}
