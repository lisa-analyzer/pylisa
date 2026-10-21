package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class RangeTest extends AnalysisTestExecutor {

	@Test
	public void testRange() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "range";
		conf.programFile = "range.py";
		perform(conf);
	}
}
