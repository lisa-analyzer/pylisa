package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ImportedGlobalsTest extends AnalysisTestExecutor {

	@Test
	public void testImportedGlobals() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "imported-globals";
		conf.programFile = "main.py";
		perform(conf);
	}
}
