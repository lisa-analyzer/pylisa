package it.unive.pylisa.cron;

import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import org.junit.Test;

public class ClassMembersTest extends AnalysisTestExecutor {

	@Test
	public void testClassMembers() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.testDir = "class-members";
		conf.programFile = "class-members.py";
		perform(conf);
	}
}
