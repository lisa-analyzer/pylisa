package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SortedTest extends AnalysisTestExecutor {

	@Test
	public void testSorted() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "sorted";
		conf.programFile = "sorted.py";
		perform(conf);
	}
}
