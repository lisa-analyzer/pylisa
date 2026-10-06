package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class PowerTest extends AnalysisTestExecutor {

	@Test
	public void testPower() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "power";
		conf.programFile = "power.py";
		perform(conf);
	}
}
