package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class BuiltinValuesTest extends AnalysisTestExecutor {

	@Test
	public void testBuiltinValues() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "builtin-values";
		conf.programFile = "builtin-values.py";
		perform(conf);
	}
}
