package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NotTest extends AnalysisTestExecutor {

	@Test
	public void testNot() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "not";
		conf.programFile = "not.py";
		perform(conf);
	}
}
