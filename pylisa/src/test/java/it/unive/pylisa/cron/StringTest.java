package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class StringTest extends AnalysisTestExecutor {

	@Test
	public void testString() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "string";
		conf.programFile = "string.py";
		perform(conf);
	}
}
