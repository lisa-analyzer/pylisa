package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ObjectInitTest extends AnalysisTestExecutor {

	@Test
	public void testObjectInit() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "object-init";
		conf.programFile = "object-init.py";
		perform(conf);
	}
}
