package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class TypeHintsTest extends AnalysisTestExecutor {

	@Test
	public void testTypeHints() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "type-hints";
		conf.programFile = "type-hints.py";
		perform(conf);
	}
}
