package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictUpdateTest extends AnalysisTestExecutor {

	@Test
	public void testDictUpdate() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "dict-update";
		conf.programFile = "dict-update.py";
		perform(conf);
	}
}
