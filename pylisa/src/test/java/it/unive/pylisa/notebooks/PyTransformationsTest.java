package it.unive.pylisa.notebooks;

import java.io.IOException;

import org.junit.Test;

import it.unive.lisa.AnalysisException;

public class PyTransformationsTest extends NotebookTest {

	@Test
	public void testCovid19() throws IOException, AnalysisException {
		perform("dataframes-tests/covid-19.py");
	}

	@Test
	public void testCreditFraud() throws IOException, AnalysisException {
		perform("dataframes-tests/credit-fraud.py");
	}

	@Test
	public void testDataExploration() throws IOException, AnalysisException {
		perform("dataframes-tests/data-exploration.py");
	}

	@Test
	public void testGuide() throws IOException, AnalysisException {
		performAndCheck("dataframes-tests/guide.py", true);
	}

	@Test
	public void testGuideSmall() throws IOException, AnalysisException {
		performAndCheck("dataframes-tests/guide-small.py");
	}

	@Test
	public void testTitanic() throws IOException, AnalysisException {
		perform("dataframes-tests/titanic.py");
	}
}
