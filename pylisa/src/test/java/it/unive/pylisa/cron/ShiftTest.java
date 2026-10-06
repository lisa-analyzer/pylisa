package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ShiftTest extends AnalysisTestExecutor {

	@Test
	public void testShift() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "shift";
		conf.programFile = "shift.py";
		perform(conf);
	}
}
