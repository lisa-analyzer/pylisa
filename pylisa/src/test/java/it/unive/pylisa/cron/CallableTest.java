package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class CallableTest extends AnalysisTestExecutor {

	@Test
	public void testCallable() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "callable";
		conf.programFile = "callable.py";
		perform(conf);
	}
}
