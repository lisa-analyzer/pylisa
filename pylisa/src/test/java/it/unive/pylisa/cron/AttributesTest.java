package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class AttributesTest extends AnalysisTestExecutor {

	@Test
	public void testAttributes() {
		CronConfiguration conf = TestHelper.constantPropagationConfig();
		conf.testDir = "attributes";
		conf.programFile = "attributes.py";
		// Reachability keeps the reachability of a guard from the first time
		// it is reached: the optimized fixpoint, reaching the guard of the
		// loop in a different order, finds its body possibly reachable rather
		// than reachable
		conf.compareWithOptimization = false;
		perform(conf);
	}
}
