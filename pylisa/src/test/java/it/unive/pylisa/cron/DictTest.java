package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictTest extends AnalysisTestExecutor {

	@Test
	public void testDict() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "dict";
		conf.programFile = "dict.py";
		perform(conf);
	}
}
