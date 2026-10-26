package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictBuiltinTest extends AnalysisTestExecutor {

	@Test
	public void testDictBuiltin() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "dict-builtin";
		conf.programFile = "dict-builtin.py";
		perform(conf);
	}
}
