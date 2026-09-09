package it.unive.pylisa.helpers;

import java.util.List;

import it.unive.lisa.conf.LiSAConfiguration;

/**
 * An extended {@link LiSAConfiguration} that also holds test configuration
 * keys. This configuration disables optimizations
 * ({@link LiSAConfiguration#optimize}) by default.
 * 
 * @author <a href="mailto:luca.negrini@unive.it">Luca Negrini</a>
 */
public class CronConfiguration extends LiSAConfiguration {

	/**
	 * The name of the test folder; this is used for searching expected results
	 * and as a working directory for executing tests in the test execution
	 * folder.
	 */
	public String testDir;

	/**
	 * An additional folder that is appended to {@link #testDir} both when
	 * computing the working directory and when searching for the expected
	 * results, but <b>not</b> for searching the source python program.
	 */
	public String testSubDir;

	/**
	 * The name of the source file to be searched in {@link #testDir}.
	 */
	public String programFile;

	/**
	 * If {@code true}, baselines will be updated if the test fails.
	 */
	public boolean forceUpdate = false;

	/**
	 * If {@code true}, marks that this run should use (or already used) an
	 * optimized fixpoint execution.
	 * <p>
	 * Note: unlike in the old LiSA API, this field does not by itself drive
	 * the analysis, since {@link LiSAConfiguration} no longer exposes an
	 * {@code optimize} toggle. It only tracks whether an optimized run has
	 * already happened, for the "run twice, once optimized" comparison logic
	 * in {@code AnalysisTestExecutor#perform(CronConfiguration)}; the actual
	 * optimization there is enabled by swapping in an
	 * {@code OptimizedForwardAscendingFixpoint} for {@link #forwardFixpoint}.
	 */
	public boolean optimize = false;

	/**
	 * If {@code true}, a second analysis will be ran with optimization enabled
	 * and the results will be checked to be equal to the non-optimized version.
	 */
	public boolean compareWithOptimization = true;

	/**
	 * In case {@link #programFile} is a python notebook, specifies a custom
	 * order of execution for its cells.
	 */
	public List<Integer> cellOrder;
}
