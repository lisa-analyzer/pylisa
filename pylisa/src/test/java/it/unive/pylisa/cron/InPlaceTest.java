package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class InPlaceTest extends AnalysisTestExecutor {

	@Test
	public void testInPlace() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "inplace";
		conf.programFile = "inplace.py";
		perform(conf);
	}
}
