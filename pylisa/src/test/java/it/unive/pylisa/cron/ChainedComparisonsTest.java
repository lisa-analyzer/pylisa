package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ChainedComparisonsTest extends AnalysisTestExecutor {

	@Test
	public void testChainedComparisons() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "chained-comparisons";
		conf.programFile = "chained-comparisons.py";
		perform(conf);
	}
}
