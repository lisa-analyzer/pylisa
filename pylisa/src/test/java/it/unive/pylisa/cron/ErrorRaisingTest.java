package it.unive.pylisa.cron;

import it.unive.lisa.triage.pdr.PdrVerifier;
import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import it.unive.pylisa.helpers.TestHelper;
import it.unive.pylisa.triage.PyExpressionEncoder;
import it.unive.pylisa.triage.PyGlobalWrites;
import it.unive.pylisa.triage.PyHeapWrites;
import org.junit.Test;

/**
 * Tests on the errors raised where the operands might have several types, or
 * several values, whose warnings are verified one by one with a
 * {@link PdrVerifier} (as the one of the SV-COMP verifier): each error is
 * raised exactly where it happens, so that its warning can be confirmed.
 */
public class ErrorRaisingTest extends AnalysisTestExecutor {

	@Test
	public void testErrorRaising() {
		CronConfiguration conf = TestHelper.svcompConfig();
		// each warning is verified (confirming it, if the error is raised
		// exactly where it happens)
		conf.warningVerifier = new PdrVerifier<>().withFixpointCheck(false).withEncoder(PyExpressionEncoder.FACTORY)
				.withGlobalWrites(new PyGlobalWrites()).withHeapWrites(new PyHeapWrites());
		conf.testDir = "error-raising";
		conf.programFile = "error-raising.py";
		perform(conf);
	}

	@Test
	public void testReviewSoundness() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.warningVerifier = new PdrVerifier<>().withFixpointCheck(false).withEncoder(PyExpressionEncoder.FACTORY)
				.withGlobalWrites(new PyGlobalWrites()).withHeapWrites(new PyHeapWrites());
		conf.testDir = "review-soundness";
		conf.programFile = "review-soundness.py";
		perform(conf);
	}

	@Test
	public void testReviewRegressions() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.warningVerifier = new PdrVerifier<>().withFixpointCheck(false).withEncoder(PyExpressionEncoder.FACTORY)
				.withGlobalWrites(new PyGlobalWrites()).withHeapWrites(new PyHeapWrites());
		conf.testDir = "review-regressions";
		conf.programFile = "review-regressions.py";
		perform(conf);
	}

	@Test
	public void testResumeAfterExpandedCall() {
		CronConfiguration conf = TestHelper.svcompConfig();
		conf.warningVerifier = new PdrVerifier<>().withFixpointCheck(false).withEncoder(PyExpressionEncoder.FACTORY)
				.withGlobalWrites(new PyGlobalWrites()).withHeapWrites(new PyHeapWrites());
		conf.testDir = "resume-split";
		conf.programFile = "resume-split.py";
		perform(conf);
	}
}
