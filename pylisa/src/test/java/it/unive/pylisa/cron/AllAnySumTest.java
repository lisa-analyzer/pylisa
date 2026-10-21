package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AllAnySumTest extends AnalysisTestExecutor {

	@Test
	public void testAllAnySum() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "all-any-sum";
		conf.programFile = "all-any-sum.py";
		perform(conf);
	}
}
