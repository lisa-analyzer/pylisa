package it.unive.pylisa.frontend;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.UnknownModuleUnit;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class PythonModuleImportManager {

	private static final Logger log = LogManager.getLogger(PythonModuleImportManager.class);

	@FunctionalInterface
	public interface ProjectFileLoader {
		ModuleUnit load(
				String moduleName,
				String filePath)
				throws IOException;
	}

	private final Program program;
	private final Map<String, ModuleUnit> loadedModules = new HashMap<>();
	private final Set<String> resolvedModules = new HashSet<>();
	private final Set<String> unknownModules = new HashSet<>();
	private CFG init;
	private final Path baseDir;
	private ProjectFileLoader projectLoader;

	public PythonModuleImportManager(
			Program program,
			CFG init,
			Path baseDir) {
		this.program = program;
		this.init = init;
		this.baseDir = baseDir;
	}

	public void setProjectLoader(
			ProjectFileLoader loader) {
		this.projectLoader = loader;
	}

	public ModuleUnit importModule(
			String moduleName) {
		if (loadedModules.containsKey(moduleName))
			return loadedModules.get(moduleName);

		ModuleUnit unit = null;

		if (LibrarySpecificationProvider.getLibraryUnit(moduleName) != null) {
			unit = LibrarySpecificationProvider.importPythonModule(program, moduleName, init);
			if (unit != null)
				resolvedModules.add(moduleName);
		}

		if (unit == null) {
			Optional<Path> projectFile = projectFileExists(moduleName);
			if (projectFile.isPresent()) {
				unit = loadProjectModule(moduleName, projectFile.get());
			}
		}

		if (unit == null) {
			unit = createUnknownModule(moduleName);
		}

		// register in PyModuleType and cache
		if (unknownModules.contains(moduleName))
			PyModuleType.registerUnknown(moduleName, unit);
		else
			PyModuleType.register(moduleName, unit);
		loadedModules.put(moduleName, unit);
		return unit;
	}

	public boolean isResolvedModule(
			String name) {
		return resolvedModules.contains(name);
	}

	public boolean isUnknownModule(
			String name) {
		return unknownModules.contains(name);
	}

	/**
	 * Returns true if {@code moduleName} corresponds to a known library spec or
	 * a project file on disk, without loading it. Used to decide whether to
	 * treat an imported name as a sub-module vs. a member.
	 */
	public boolean canResolveModule(
			String moduleName) {
		if (loadedModules.containsKey(moduleName))
			return resolvedModules.contains(moduleName);
		if (LibrarySpecificationProvider.getLibraryUnit(moduleName) != null)
			return true;
		return projectFileExists(moduleName).isPresent();
	}

	private Optional<Path> projectFileExists(
			String moduleName) {
		if (baseDir == null)
			return Optional.empty();
		String pathStr = moduleName.replace('.', '/');
		Path filePath = baseDir.resolve(pathStr + ".py");
		if (filePath.toFile().exists())
			return Optional.of(filePath);
		Path initPath = baseDir.resolve(pathStr + "/__init__.py");
		if (initPath.toFile().exists())
			return Optional.of(initPath);
		return Optional.empty();
	}

	private ModuleUnit loadProjectModule(
			String moduleName,
			Path filePath) {
		// Create and register BEFORE parsing to prevent circular import
		// re-entry
		ModuleUnit unit = new ModuleUnit(
				SyntheticLocation.INSTANCE, program, moduleName);
		program.addUnit(unit);
		PyModuleType.register(moduleName, unit);
		loadedModules.put(moduleName, unit);
		resolvedModules.add(moduleName);
		unknownModules.remove(moduleName);

		if (projectLoader != null) {
			try {
				projectLoader.load(moduleName, filePath.toString());
			} catch (Exception e) {
				// log but continue — unit is already registered as partial stub
				log.warn("[PyLiSA] Failed to load project module " + moduleName
						+ ": " + e.getClass().getSimpleName() + ": " + e.getMessage());
			}
		}
		return unit;
	}

	private ModuleUnit createUnknownModule(
			String moduleName) {
		UnknownModuleUnit unit = new UnknownModuleUnit(
				SyntheticLocation.INSTANCE, program, moduleName);
		PyCFG initModule = new PyCFG(
				new CodeMemberDescriptor(SyntheticLocation.INSTANCE, unit, false, "__initmodule__"));
		unit.addCodeMember(initModule);
		PyCFG initNoOp = new PyCFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, unit, false, "$init"));
		initNoOp.addNode(new it.unive.lisa.program.cfg.statement.Ret(initNoOp, SyntheticLocation.INSTANCE), true);
		unit.addCodeMember(initNoOp);
		unknownModules.add(moduleName);

		if (it.unive.pylisa.cfg.type.PyClassType.isRegistered("builtins.object"))
			unit.addAncestor(it.unive.pylisa.cfg.type.PyClassType.lookup("builtins.object").getUnit());

		program.addUnit(unit);
		return unit;
	}
}
