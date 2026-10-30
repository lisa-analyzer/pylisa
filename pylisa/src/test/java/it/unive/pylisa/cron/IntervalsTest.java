package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class IntervalsTest extends AnalysisTestExecutor {

	@Test
	public void testIntervals() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "intervals";
		conf.programFile = "intervals.py";
		perform(conf);
	}
}
