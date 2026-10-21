package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class IsInstanceTest extends AnalysisTestExecutor {

	@Test
	public void testIsInstance() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "isinstance";
		conf.programFile = "isinstance.py";
		perform(conf);
	}
}
