package it.unive.pylisa.frontend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.pylisa.program.UnknownModuleUnit;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Checks that modules that are neither project files nor library
 * specifications can be supplied through module providers and search paths.
 */
class ModuleProviderTest {

	@TempDir
	Path project;

	@TempDir
	Path extra;

	@Test
	void aModuleFoundOnTheSearchPathIsParsedLikeAProjectFile() throws Exception {
		Files.createDirectories(extra.resolve("extra_pkg/msg"));
		Files.writeString(extra.resolve("extra_pkg/msg/__init__.py"), "class Thing:\n    pass\n");
		Path program = write("from extra_pkg.msg import Thing\nt = Thing()\n");

		Program translated = new PyFrontend(program.toString(), false)
				.addModuleSearchPath(extra)
				.toLiSAProgram(true);

		Unit module = translated.getUnit("extra_pkg.msg");
		assertNotNull(module);
		assertFalse(module instanceof UnknownModuleUnit, "the module was not taken from the search path");
	}

	@Test
	void providersAreAskedOnlyForModulesTheyDoNotShadow() throws Exception {
		Path program = write("import json\nimport generated_mod\n");
		Path generated = extra.resolve("generated_mod.py");
		Files.writeString(generated, "VALUE = 1\n");
		List<String> asked = new ArrayList<>();

		Program translated = new PyFrontend(program.toString(), false)
				.addModuleProvider(name -> {
					asked.add(name);
					return name.equals("generated_mod") ? Optional.of(generated) : Optional.empty();
				})
				.toLiSAProgram(true);

		assertEquals(List.of("generated_mod"), asked, "a library module was asked to the provider");
		assertFalse(translated.getUnit("generated_mod") instanceof UnknownModuleUnit);
	}

	@Test
	void aModuleNoProviderKnowsIsUnknown() throws Exception {
		Path program = write("import nowhere_to_be_found\n");

		Program translated = new PyFrontend(program.toString(), false)
				.addModuleSearchPath(extra)
				.toLiSAProgram(true);

		assertTrue(translated.getUnit("nowhere_to_be_found") instanceof UnknownModuleUnit);
	}

	private Path write(
			String source)
			throws Exception {
		Path file = project.resolve("main.py");
		Files.writeString(file, source);
		return file;
	}
}
