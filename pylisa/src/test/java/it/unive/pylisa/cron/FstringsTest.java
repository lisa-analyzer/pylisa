package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class FstringsTest extends AnalysisTestExecutor {

	@Test
	public void testFstrings() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "fstrings";
		conf.programFile = "fstrings.py";
		perform(conf);
	}
}
