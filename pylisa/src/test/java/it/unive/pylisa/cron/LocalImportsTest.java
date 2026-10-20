package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class LocalImportsTest extends AnalysisTestExecutor {

	@Test
	public void testLocalImports() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "local-imports";
		conf.programFile = "main.py";
		perform(conf);
	}
}
