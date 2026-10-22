package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NondetFloatRangeTest extends AnalysisTestExecutor {

	@Test
	public void testNondetFloatRange() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "nondet-float-range";
		conf.programFile = "nondet-float-range.py";
		perform(conf);
	}
}
