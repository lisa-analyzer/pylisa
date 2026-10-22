package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class TryExceptTest extends AnalysisTestExecutor {

	@Test
	public void testTryExcept() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "try-except";
		conf.programFile = "try-except.py";
		perform(conf);
	}
}
