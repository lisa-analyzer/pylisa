package it.unive.pylisa.notebooks;

import it.unive.lisa.AnalysisException;
import java.io.IOException;
import org.junit.Test;

public class JupyterTransformationsTest extends NotebookTest {

	@Test
	public void testCovid19() throws IOException, AnalysisException {
		perform("dataframes-tests/notebook/covid-19.ipynb");
	}

	@Test
	public void testCreditFraud() throws IOException, AnalysisException {
		perform("dataframes-tests/notebook/credit-fraud.ipynb");
	}

	@Test
	public void testDataExploration() throws IOException, AnalysisException {
		perform("dataframes-tests/notebook/data-exploration.ipynb");
	}

	@Test
	public void testTitanic() throws IOException, AnalysisException {
		perform("dataframes-tests/notebook/titanic.ipynb");
	}
}
