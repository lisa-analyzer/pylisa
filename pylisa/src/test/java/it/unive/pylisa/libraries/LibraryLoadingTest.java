package it.unive.pylisa.libraries;

import static org.junit.Assert.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import java.io.File;
import org.junit.Test;

public class LibraryLoadingTest {

	@Test
	public void ensureAllLoaded() {
		String[] allLibs = new File("src/main/resources" + LibrarySpecificationProvider.LIBS_FOLDER).list();
		Program p = new Program(new PythonFeatures(), new PythonTypeSystem());
		LibrarySpecificationProvider.load(p, makeInit(p));
		// -1 since it also contains stdlib that is not included in the
		// available libraries
		// <= since each file contains at least one library
		assertTrue(allLibs.length - 1 <= LibrarySpecificationProvider.getLibraryUnits().size());
	}

	private CFG makeInit(
			Program program) {
		CFG init = new CFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, program, false, "LiSA$init"));
		init.addNode(new Ret(init, SyntheticLocation.INSTANCE), true);
		program.addCodeMember(init);
		return init;
	}
}
