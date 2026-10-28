package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ListPopExtendTest extends AnalysisTestExecutor {

	@Test
	public void testListPopExtend() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "list-pop-extend";
		conf.programFile = "list-pop-extend.py";
		perform(conf);
	}
}
