package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BoundMethodsTest extends AnalysisTestExecutor {

	@Test
	public void testBoundMethods() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "bound-methods";
		conf.programFile = "bound-methods.py";
		perform(conf);
	}
}
