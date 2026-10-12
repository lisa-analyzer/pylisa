package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class FunctionValuesTest extends AnalysisTestExecutor {

	@Test
	public void testFunctionValues() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "function-values";
		conf.programFile = "function-values.py";
		perform(conf);
	}
}
