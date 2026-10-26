package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ListConcatenationTest extends AnalysisTestExecutor {

	@Test
	public void testListConcatenation() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "list-concatenation";
		conf.programFile = "list-concatenation.py";
		perform(conf);
	}
}
