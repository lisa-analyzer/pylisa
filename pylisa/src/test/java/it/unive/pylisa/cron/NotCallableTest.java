package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NotCallableTest extends AnalysisTestExecutor {

	@Test
	public void testNotCallable() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "not-callable";
		conf.programFile = "not-callable.py";
		perform(conf);
	}
}
