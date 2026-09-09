package it.unive.pylisa.notebooks;

import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.cfg.fixpoints.optforward.OptimizedForwardAscendingFixpoint;
import it.unive.pylisa.analysis.dataframes.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.DataframeGraphValueDomain;
import it.unive.pylisa.checks.BottomFinder;
import it.unive.pylisa.checks.DataframeDumper;
import it.unive.pylisa.checks.DataframeStructureConstructor;
import it.unive.pylisa.checks.OpenCallsFinder;
import it.unive.pylisa.helpers.AnalysisTestExecutor;
import it.unive.pylisa.helpers.CronConfiguration;
import org.junit.Ignore;
import org.junit.Test;

@Ignore
public class PyTransformationsTest extends AnalysisTestExecutor {

	private CronConfiguration buildConfig() {
		return buildConfig(false);
	}

	private CronConfiguration buildConfig(
			boolean findOpenCalls) {
		CronConfiguration conf = new CronConfiguration();
		// conf.outputs.add(new HtmlResults<>(true));
		conf.optimize = true;
		conf.forwardFixpoint = new OptimizedForwardAscendingFixpoint<>();
		conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
		conf.callGraph = new RTACallGraph();
		conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
		conf.semanticChecks.add(new DataframeDumper());
		if (!conf.optimize)
			// if optimize is true, we will have bottom almost everywhere
			conf.semanticChecks.add(new BottomFinder());
		conf.semanticChecks.add(new DataframeStructureConstructor());
		if (findOpenCalls)
			conf.semanticChecks.add(new OpenCallsFinder<>());

		conf.analysis = new SimpleAbstractDomain<HeapEnvWithFields, DataframeGraphDomain, TypeEnvironment<TypeSet>>(
				new FieldSensitivePointBasedHeap(), new DataframeGraphValueDomain(), new InferredTypes());

		conf.compareWithOptimization = false;

		return conf;
	}

	@Test
	public void testCovid19() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/covid-19";
		conf.programFile = "covid-19.ipynb";
		perform(conf);
	}

	@Test
	public void testCreditFraud() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/credit-fraud";
		conf.programFile = "credit-fraud.ipynb";
		perform(conf);
	}

	@Test
	public void testDataExploration() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/data-exploration";
		conf.programFile = "data-exploration.ipynb";
		perform(conf);
	}

	@Test
	public void testTitanic() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/titanic";
		conf.programFile = "titanic.ipynb";
		perform(conf);
	}

	@Test
	public void testCovid19Updated() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/covid-19-updated";
		conf.programFile = "covid-19-updated.py";
		perform(conf);
	}

	@Test
	public void testGuide() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/guide";
		conf.programFile = "guide.py";
		perform(conf);
	}

	@Test
	public void testGuideSmall() {
		CronConfiguration conf = buildConfig();
		conf.testDir = "notebooks/guide-small";
		conf.programFile = "guide-small.py";
		perform(conf);
	}
}
