package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class NamedTupleTest extends AnalysisTestExecutor {

	@Test
	public void testNamedTuple() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "namedtuple";
		conf.programFile = "namedtuple.py";
		perform(conf);
	}
}
