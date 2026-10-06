package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class StringIndexTest extends AnalysisTestExecutor {

	@Test
	public void testStringIndex() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "string-index";
		conf.programFile = "string-index.py";
		perform(conf);
	}
}
