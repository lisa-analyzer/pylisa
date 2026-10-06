package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class StringFormatTest extends AnalysisTestExecutor {

	@Test
	public void testStringFormat() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "string-format";
		conf.programFile = "string-format.py";
		perform(conf);
	}
}
