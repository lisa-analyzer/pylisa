package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictUnpackingTest extends AnalysisTestExecutor {

	@Test
	public void testDictUnpacking() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "dict-unpacking";
		conf.programFile = "dict-unpacking.py";
		perform(conf);
	}
}
