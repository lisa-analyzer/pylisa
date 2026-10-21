package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class DictViewsTest extends AnalysisTestExecutor {

	@Test
	public void testDictViews() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "dict-views";
		conf.programFile = "dict-views.py";
		// the tuples of d.items() are allocated while iterating: in a loop, the
		// unwinding of the optimized fixpoint does not reproduce the states of
		// the first iterations, where their allocation site is still strong
		conf.compareWithOptimization = false;
		perform(conf);
	}
}
