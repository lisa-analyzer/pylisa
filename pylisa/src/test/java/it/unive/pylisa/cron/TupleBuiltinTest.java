package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class TupleBuiltinTest extends AnalysisTestExecutor {

	@Test
	public void testTupleBuiltin() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "tuple-builtin";
		conf.programFile = "tuple-builtin.py";
		perform(conf);
	}
}
