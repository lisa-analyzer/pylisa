package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ClassCallsTest extends AnalysisTestExecutor {

	@Test
	public void testClassCalls() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "class-calls";
		conf.programFile = "class-calls.py";
		perform(conf);
	}
}
