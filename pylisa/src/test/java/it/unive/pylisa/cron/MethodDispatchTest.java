package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class MethodDispatchTest extends AnalysisTestExecutor {

	@Test
	public void testMethodDispatch() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "method-dispatch";
		conf.programFile = "method-dispatch.py";
		// B.func is reached through super() in C.func from two calls, on
		// objects of different allocation sites: the unwinding of the
		// optimized fixpoint computes its states from the entry states of
		// both calls, while the regular fixpoint keeps them apart
		conf.compareWithOptimization = false;
		perform(conf);
	}
}
