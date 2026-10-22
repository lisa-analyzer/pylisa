package it.unive.pylisa;

import static org.junit.Assert.assertEquals;

import java.nio.file.Path;
import org.junit.Test;

public class LocalModulesTest {

	private static final Path ROOT = Path.of("prog");

	@Test
	public void modulesAreNamedRelativeToTheRoot() {
		assertEquals("main", LocalModules.moduleName(ROOT.resolve("main.py"), ROOT));
		assertEquals("pkg.sub", LocalModules.moduleName(ROOT.resolve("pkg/sub.py"), ROOT));
		assertEquals("notebook", LocalModules.moduleName(ROOT.resolve("notebook.ipynb"), ROOT));
	}

	@Test
	public void packagesAreNamedAfterTheirDirectory() {
		assertEquals("pkg", LocalModules.moduleName(ROOT.resolve("pkg/__init__.py"), ROOT));
		assertEquals("pkg.inner", LocalModules.moduleName(ROOT.resolve("pkg/inner/__init__.py"), ROOT));
	}

	@Test
	public void filesOutsideTheRootUseTheirUnitName() {
		Path outside = Path.of("other", "mod.py");
		assertEquals(LocalModules.unit(outside), LocalModules.moduleName(outside, ROOT));
		Path init = ROOT.resolve("__init__.py");
		assertEquals(LocalModules.unit(init), LocalModules.moduleName(init, ROOT));
	}
}
