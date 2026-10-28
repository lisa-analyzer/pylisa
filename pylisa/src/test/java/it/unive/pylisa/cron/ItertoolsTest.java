package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ItertoolsTest extends AnalysisTestExecutor {

	@Test
	public void testItertools() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "itertools";
		conf.programFile = "itertools-functions.py";
		perform(conf);
	}
}
