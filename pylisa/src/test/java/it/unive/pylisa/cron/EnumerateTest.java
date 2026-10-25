package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class EnumerateTest extends AnalysisTestExecutor {

	@Test
	public void testEnumerate() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "enumerate";
		conf.programFile = "enumerate.py";
		perform(conf);
	}
}
