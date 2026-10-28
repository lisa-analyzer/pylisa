package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SubscriptCallsTest extends AnalysisTestExecutor {

	@Test
	public void testSubscriptCalls() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "subscript-calls";
		conf.programFile = "subscript-calls.py";
		perform(conf);
	}
}
