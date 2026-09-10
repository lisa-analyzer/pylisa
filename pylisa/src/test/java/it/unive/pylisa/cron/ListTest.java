package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ListTest extends AnalysisTestExecutor {

	@Test
	public void testList() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "list";
		conf.programFile = "list.py";
		perform(conf);
	}
}
