package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class StarImportTest extends AnalysisTestExecutor {

	@Test
	public void testStarImport() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "star-import";
		conf.programFile = "main.py";
		perform(conf);
	}
}
