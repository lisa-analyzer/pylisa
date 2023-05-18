package it.unive.pylisa.notebooks;

import it.unive.lisa.AnalysisException;
import java.io.IOException;
import org.junit.Test;

public class PyTransformationsTest extends NotebookTest {

	@Test
	public void testCovid19() throws IOException, AnalysisException {
		performAndCheck("dataframes-tests/covid-19.ipynb");
	}

	@Test
	public void testCreditFraud() throws IOException, AnalysisException {
		perform("dataframes-tests/credit-fraud.ipynb");
	}

	@Test
	public void testDataExploration() throws IOException, AnalysisException {
		perform("dataframes-tests/data-exploration.ipynb");
	}

	@Test
	public void testTitanic() throws IOException, AnalysisException {
		perform("dataframes-tests/titanic.ipynb");
	}

	@Test
	public void testCovid19Updated() throws IOException, AnalysisException {
		performAndCheck("dataframes-tests/covid-19-updated.py");
	}

	@Test
	public void testGuide() throws IOException, AnalysisException {
		performAndCheck("dataframes-tests/guide.py");
	}

	@Test
	public void testGuideSmall() throws IOException, AnalysisException {
		performAndCheck("dataframes-tests/guide-small.py");
	}
}
