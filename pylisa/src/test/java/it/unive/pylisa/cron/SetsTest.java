package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SetsTest extends AnalysisTestExecutor {

	@Test
	public void testSets() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "sets";
		conf.programFile = "sets.py";
		perform(conf);
	}
}
