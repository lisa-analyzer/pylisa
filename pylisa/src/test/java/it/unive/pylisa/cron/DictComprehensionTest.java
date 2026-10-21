package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictComprehensionTest extends AnalysisTestExecutor {

	@Test
	public void testDictComprehension() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "dict-comprehension";
		conf.programFile = "dict-comprehension.py";
		perform(conf);
	}
}
