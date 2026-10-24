package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class WeakAllocationsTest extends AnalysisTestExecutor {

	@Test
	public void testWeakAllocations() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "weak-allocations";
		conf.programFile = "weak-allocations.py";
		perform(conf);
	}
}
