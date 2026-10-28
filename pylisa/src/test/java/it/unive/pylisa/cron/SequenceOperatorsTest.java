package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SequenceOperatorsTest extends AnalysisTestExecutor {

	@Test
	public void testSequenceOperators() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "sequence-operators";
		conf.programFile = "sequence-operators.py";
		perform(conf);
	}
}
