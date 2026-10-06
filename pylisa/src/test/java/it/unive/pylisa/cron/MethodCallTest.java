package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MethodCallTest extends AnalysisTestExecutor {

	@Test
	public void testMethodCall() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "method-call";
		conf.programFile = "method-call.py";
		perform(conf);
	}
}
