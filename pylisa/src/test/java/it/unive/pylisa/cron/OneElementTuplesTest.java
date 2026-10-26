package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class OneElementTuplesTest extends AnalysisTestExecutor {

	@Test
	public void testOneElementTuples() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "one-element-tuples";
		conf.programFile = "one-element-tuples.py";
		perform(conf);
	}
}
