package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class SliceAssignmentTest extends AnalysisTestExecutor {

	@Test
	public void testSliceAssignment() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "slice-assignment";
		conf.programFile = "slice-assignment.py";
		perform(conf);
	}
}
