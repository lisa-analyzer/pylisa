package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class LoopIntervalsTest extends AnalysisTestExecutor {

	@Test
	public void testLoopIntervals() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "loop-intervals";
		conf.programFile = "loop-intervals.py";
		perform(conf);
	}
}
