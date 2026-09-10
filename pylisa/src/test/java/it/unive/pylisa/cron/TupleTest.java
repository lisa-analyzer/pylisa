package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class TupleTest extends AnalysisTestExecutor {

	@Test
	public void testTuple() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "tuple";
		conf.programFile = "tuple.py";
		perform(conf);
	}
}
