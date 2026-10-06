package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class StringMethodsTest extends AnalysisTestExecutor {

	@Test
	public void testStringMethods() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "string-methods";
		conf.programFile = "string-methods.py";
		perform(conf);
	}
}
