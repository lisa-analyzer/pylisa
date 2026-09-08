package it.unive.pylisa.libraries;

import static org.junit.Assert.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import java.io.File;
import org.junit.Test;

public class LibraryLoadingTest {

	@Test
	public void ensureAllLoaded() {
		String[] allLibs = new File("src/main/resources" + LibrarySpecificationProvider.LIBS_FOLDER).list();
		Program p = new Program(new PythonFeatures(), new PythonTypeSystem());
		LibrarySpecificationProvider.load(p);
		// -2: stdlib.txt and int.txt only augment always-available built-in
		// types (via bare 'class' declarations) and are not included in the
		// available libraries, unlike e.g. numpy.txt or pandas.txt which
		// wrap their content in a 'library ...: location ...' block
		// <= since each of the remaining files contains at least one library
		assertTrue(allLibs.length - 2 <= LibrarySpecificationProvider.getLibraryUnits().size());
	}
}
