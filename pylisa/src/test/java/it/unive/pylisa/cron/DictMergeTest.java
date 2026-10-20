package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictMergeTest extends AnalysisTestExecutor {

	@Test
	public void testDictMerge() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "dict-merge";
		conf.programFile = "dict-merge.py";
		perform(conf);
	}
}
