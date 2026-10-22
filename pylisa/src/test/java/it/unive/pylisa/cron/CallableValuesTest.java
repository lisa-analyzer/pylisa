package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class CallableValuesTest extends AnalysisTestExecutor {

	@Test
	public void testCallableValues() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "callable-values";
		conf.programFile = "callable-values.py";
		perform(conf);
	}
}
