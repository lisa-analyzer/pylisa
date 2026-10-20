package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MinMaxAbsTest extends AnalysisTestExecutor {

	@Test
	public void testMinMaxAbs() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "min-max-abs";
		conf.programFile = "min-max-abs.py";
		perform(conf);
	}
}
