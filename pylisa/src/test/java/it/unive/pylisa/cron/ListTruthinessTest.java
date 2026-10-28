package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ListTruthinessTest extends AnalysisTestExecutor {

	@Test
	public void testListTruthiness() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "list-truthiness";
		conf.programFile = "list-truthiness.py";
		perform(conf);
	}
}
