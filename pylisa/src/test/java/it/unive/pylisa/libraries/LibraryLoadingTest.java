package it.unive.pylisa.libraries;

import static org.junit.Assert.assertTrue;

import java.io.File;

import org.junit.Test;

import it.unive.lisa.program.Program;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;

public class LibraryLoadingTest {

	@Test
	public void ensureAllLoaded() {
		String[] allLibs = new File("src/main/resources" + LibrarySpecificationProvider.LIBS_FOLDER).list();
		Program p = new Program(new PythonFeatures(), new PythonTypeSystem());
		LibrarySpecificationProvider.load(p);
		// -1 since it also contains stdlib that is not included in the
		// available libraries
		// <= since each file contains at least one library
		assertTrue(allLibs.length - 1 <= LibrarySpecificationProvider.getLibraryUnits().size());
	}
}
