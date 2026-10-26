package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class PackageAliasesTest extends AnalysisTestExecutor {

	@Test
	public void testPackageAliases() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "package-aliases";
		conf.programFile = "main.py";
		perform(conf);
	}
}
