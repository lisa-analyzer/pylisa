package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SortedKeyTest extends AnalysisTestExecutor {

	@Test
	public void testSortedKey() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "sorted-key";
		conf.programFile = "sorted-key.py";
		perform(conf);
	}
}
