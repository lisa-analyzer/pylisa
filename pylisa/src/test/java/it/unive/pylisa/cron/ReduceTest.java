package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ReduceTest extends AnalysisTestExecutor {

	@Test
	public void testReduce() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "reduce";
		conf.programFile = "reduce.py";
		perform(conf);
	}
}
