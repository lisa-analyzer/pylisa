package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ItemAssignmentTest extends AnalysisTestExecutor {

	@Test
	public void testItemAssignment() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "item-assignment";
		conf.programFile = "item-assignment.py";
		perform(conf);
	}
}
