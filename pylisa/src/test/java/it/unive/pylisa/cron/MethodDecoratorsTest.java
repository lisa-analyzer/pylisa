package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MethodDecoratorsTest extends AnalysisTestExecutor {

	@Test
	public void testMethodDecorators() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "method-decorators";
		conf.programFile = "method-decorators.py";
		perform(conf);
	}
}
