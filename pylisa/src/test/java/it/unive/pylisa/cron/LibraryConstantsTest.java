package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class LibraryConstantsTest extends AnalysisTestExecutor {

	@Test
	public void testLibraryConstants() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "library-constants";
		conf.programFile = "library-constants.py";
		perform(conf);
	}
}
