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
	// Maps canonical file path → the unit already loaded for that file. Used
	// to unify modules imported under different names (e.g. "config" vs.
	// "dispatch.config" both pointing to dispatch/config.py).
	private final Map<Path, ModuleUnit> fileToUnit = new HashMap<>();
	private final Set<String> resolvedModules = new HashSet<>();
	private final Set<String> unknownModules = new HashSet<>();
	private CFG init;
	private final Path baseDir;
	// Models Python's sys.path root. When baseDir is itself inside a package
	// (i.e. has an __init__.py), the true sys.path entry is its parent — a
	// bare `import X` resolves against syspathRoot, not baseDir, matching
	// Python 3 absolute-import semantics.
	private final Path syspathRoot;
	// Dotted package name the entry file belongs to, derived by walking up
	// from baseDir while each parent has an __init__.py. Null when baseDir
	// is not inside a package. Used to supply a __package__-like anchor for
	// relative imports in the entry file, whose currentModule is the bare
	// "__main__" and therefore carries no package context.
	private final String packageName;
	private ProjectFileLoader projectLoader;

	public PythonModuleImportManager(
			Program program,
			CFG init,
			Path baseDir) {
		this.program = program;
		this.init = init;
		this.baseDir = baseDir;
		if (baseDir != null && baseDir.resolve("__init__.py").toFile().exists()) {
			java.util.List<String> parts = new java.util.ArrayList<>();
			Path cur = baseDir;
			while (cur != null && cur.resolve("__init__.py").toFile().exists()) {
				Path name = cur.getFileName();
				if (name == null)
					break;
				parts.add(0, name.toString());
				cur = cur.getParent();
			}
			this.packageName = parts.isEmpty() ? null : String.join(".", parts);
			this.syspathRoot = cur != null ? cur : baseDir;
		} else {
			this.packageName = null;
			this.syspathRoot = baseDir;
		}
	}

	/**
	 * Yields the dotted package name that {@code baseDir} belongs to, or
	 * {@code null} if the entry file is not inside a package. Used by the
	 * statement visitor's relative-import resolver as a fallback when the
	 * current module name ({@code "__main__"} for the entry file) does not
	 * itself carry the package prefix.
	 *
	 * @return the package name, or {@code null}
	 */
	public String getPackageName() {
		return packageName;
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

		// Python-accurate order: sys.path[0] (local project files) beats the
		// library spec (stdlib / FastAPI / Flask / …). A local `random/`
		// package correctly shadows stdlib `random`, as it does at runtime.
		Optional<Path> projectFile = projectFileExists(moduleName);
		if (projectFile.isPresent()) {
			// Deduplicate: if this file was already loaded under a different
			// module name, reuse the existing unit rather than parsing it
			// twice. This prevents e.g. "config" and "dispatch.config" both
			// pointing to the same config.py from producing two independent
			// CBA analysis contexts that ping-pong forever.
			Path canonical = projectFile.get().toAbsolutePath().normalize();
			ModuleUnit existing = fileToUnit.get(canonical);
			if (existing != null) {
				unit = existing;
			} else {
				unit = loadProjectModule(moduleName, projectFile.get());
			}
		}

		if (unit == null && LibrarySpecificationProvider.getLibraryUnit(moduleName) != null) {
			unit = LibrarySpecificationProvider.importPythonModule(program, moduleName, init);
			if (unit != null)
				resolvedModules.add(moduleName);
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

		// Python implicitly imports every parent package when loading a
		// submodule — `import x.y.z` populates sys.modules with "x", "x.y",
		// and "x.y.z". We match that at the type level for *unresolvable*
		// ancestors only: registering them as UnknownModuleUnits preserves
		// the guarantee that each prefix is a module/package. We intentionally
		// do NOT trigger parsing of resolvable ancestors' __init__.py files
		// here — doing so can cascade expensively and is best left to an
		// explicit import.
		registerUnknownAncestors(moduleName);
		return unit;
	}

	private void registerUnknownAncestors(
			String moduleName) {
		int idx = moduleName.indexOf('.');
		while (idx > 0) {
			String ancestor = moduleName.substring(0, idx);
			if (!loadedModules.containsKey(ancestor)
					&& LibrarySpecificationProvider.getLibraryUnit(ancestor) == null
					&& projectFileExists(ancestor).isEmpty()) {
				ModuleUnit u = createUnknownModule(ancestor);
				PyModuleType.registerUnknown(ancestor, u);
				loadedModules.put(ancestor, u);
			}
			idx = moduleName.indexOf('.', idx + 1);
		}
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
		// Mirror importModule's Python-accurate order: local file first,
		// library spec second.
		if (projectFileExists(moduleName).isPresent())
			return true;
		return LibrarySpecificationProvider.getLibraryUnit(moduleName) != null;
	}

	private Optional<Path> projectFileExists(
			String moduleName) {
		if (syspathRoot == null)
			return Optional.empty();
		// Absolute lookup from the sys.path root. When baseDir is inside a
		// package, syspathRoot is its parent — so a bare `import X` never
		// reaches a sibling `X.py` in the current package (which is only
		// reachable via the qualified name or a relative `from .X import …`).
		String rel = moduleName.replace('.', '/');
		Path file = syspathRoot.resolve(rel + ".py");
		if (file.toFile().exists())
			return Optional.of(file);
		Path pkg = syspathRoot.resolve(rel + "/__init__.py");
		if (pkg.toFile().exists())
			return Optional.of(pkg);
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
		// Track canonical path so that alias imports (e.g. "dispatch.config"
		// after "config") can reuse this unit instead of creating a duplicate.
		fileToUnit.put(filePath.toAbsolutePath().normalize(), unit);

		if (projectLoader != null) {
			try {
				projectLoader.load(moduleName, filePath.toString());
			} catch (Exception e) {
				// log but continue — unit is already registered as partial stub
				log.warn("[PyLiSA] Failed to load project module " + moduleName
						+ ": " + e.getClass().getSimpleName() + ": " + e.getMessage(), e);
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
