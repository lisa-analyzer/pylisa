package it.unive.pylisa;

import it.unive.pylisa.antlr.PythonParser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_fromContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_nameContext;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.io.FilenameUtils;

/**
 * The local modules imported by a file: the python files in the directory of
 * the analyzed program (where python looks for the modules imported with
 * absolute names), or in the package of the importing file (for relative
 * imports, as {@code from . import m}). A module {@code a.b} is the file
 * {@code a/b.py}, or the package {@code a/b/__init__.py}. The libraries that
 * are modeled (e.g. {@code _sv_verifier}) are never local modules.
 */
final class LocalModules {

	/**
	 * The imports of a file.
	 *
	 * @param bindings     the names they bind to local modules
	 * @param dependencies the files of the local modules they import, in the
	 *                         order of the imports
	 */
	record Imports(
			LocalImports bindings,
			List<Path> dependencies) {
	}

	private LocalModules() {
	}

	/**
	 * Yields the imports of local modules of a file.
	 *
	 * @param tree the parse tree of the file
	 * @param file the file
	 * @param root the directory of the analyzed program
	 *
	 * @return the imports
	 */
	static Imports of(
			ParseTree tree,
			Path file,
			Path root) {
		Map<String, String> modules = new LinkedHashMap<>();
		Map<String, LocalImports.Member> members = new LinkedHashMap<>();
		List<Path> dependencies = new ArrayList<>();
		List<ParserRuleContext> imports = new ArrayList<>();
		collect(tree, imports);

		for (ParserRuleContext imp : imports)
			if (imp instanceof Import_nameContext)
				for (Dotted_as_nameContext name : ((Import_nameContext) imp).dotted_as_names().dotted_as_name()) {
					List<String> parts = parts(name.dotted_name());
					if (name.name() != null) {
						// import a.b as x binds x to a.b
						Path module = resolve(root, file, 0, String.join(".", parts));
						if (module != null) {
							modules.put(name.name().getText(), unit(module));
							dependencies.add(module);
						}
					} else
						// import a.b binds a, and a.b is its attribute b
						for (int i = 1; i <= parts.size(); i++) {
							String prefix = String.join(".", parts.subList(0, i));
							Path module = resolve(root, file, 0, prefix);
							if (module != null) {
								modules.put(prefix, unit(module));
								dependencies.add(module);
							}
						}
				}
			else {
				Import_fromContext from = (Import_fromContext) imp;
				if (from.import_from_targets().import_from_as_names() == null)
					// from m import *
					continue;
				// '...' is a single token
				int level = from.DOT().size() + 3 * from.ELLIPSIS().size();
				String name = from.dotted_name() == null ? "" : String.join(".", parts(from.dotted_name()));
				Path module = resolve(root, file, level, name);
				if (module != null)
					dependencies.add(module);
				for (Import_from_as_nameContext target : from.import_from_targets().import_from_as_names()
						.import_from_as_name()) {
					String member = target.name(0).getText();
					String local = target.name().size() == 2 ? target.name(1).getText() : member;
					// from m import x: x is either a submodule of m, or a
					// member
					// of m
					Path submodule = resolve(root, file, level, name.isEmpty() ? member : name + "." + member);
					if (submodule != null) {
						modules.put(local, unit(submodule));
						dependencies.add(submodule);
					} else if (module != null)
						members.put(local, new LocalImports.Member(unit(module), moduleName(module, root), member));
				}
			}

		Map<String, String> names = new LinkedHashMap<>();
		for (Path dependency : dependencies)
			names.put(unit(dependency), moduleName(dependency, root));
		return new Imports(new LocalImports(modules, members, names), dependencies);
	}

	// the import statements in the tree, in order
	private static void collect(
			ParseTree tree,
			List<ParserRuleContext> imports) {
		if (tree instanceof Import_nameContext || tree instanceof Import_fromContext) {
			imports.add((ParserRuleContext) tree);
			return;
		}
		for (int i = 0; i < tree.getChildCount(); i++)
			collect(tree.getChild(i), imports);
	}

	private static List<String> parts(
			Dotted_nameContext name) {
		List<String> parts = new ArrayList<>();
		for (Dotted_nameContext n = name; n != null; n = n.dotted_name())
			parts.add(0, n.name().getText());
		return parts;
	}

	/**
	 * Yields the file of the module with the given name, imported by the file
	 * with the given relative level (the number of leading dots, 0 for an
	 * absolute import).
	 *
	 * @param root  the directory of the analyzed program
	 * @param file  the importing file
	 * @param level the level of the import
	 * @param name  the dotted name of the module (empty for the package of a
	 *                  relative import, as in {@code from . import x})
	 *
	 * @return the file, or {@code null} if it is not a local module
	 */
	static Path resolve(
			Path root,
			Path file,
			int level,
			String name) {
		if (level == 0 && LibrarySpecificationProvider.getLibraryUnit(name.split("\\.")[0]) != null)
			// a library that is modeled
			return null;
		Path base = level == 0 ? root : directory(file);
		for (int i = 1; i < level; i++)
			// from .. import x: the parent package
			base = directory(base);
		Path path = name.isEmpty() ? base : base.resolve(name.replace('.', '/'));
		if (!name.isEmpty()) {
			Path module = path.resolveSibling(path.getFileName() + ".py");
			if (Files.isRegularFile(module))
				return module.normalize();
		}
		Path init = path.resolve("__init__.py");
		if (Files.isRegularFile(init))
			return init.normalize();
		return null;
	}

	// the directory containing the given path (the current one if it has no
	// parent)
	private static Path directory(
			Path path) {
		Path parent = path.getParent();
		return parent == null ? Path.of("") : parent;
	}

	/**
	 * Yields the (dotted) name of the module in the given file, relative to the
	 * directory of the analyzed program: {@code a/b.py} is module {@code a.b},
	 * and {@code a/b/__init__.py} is the package {@code a.b}. If the file is
	 * not inside the directory of the program, the name of its unit (see
	 * {@link #unit(Path)}) is used instead.
	 *
	 * @param file the file
	 * @param root the directory of the analyzed program
	 *
	 * @return the name of the module
	 */
	static String moduleName(
			Path file,
			Path root) {
		Path rel = root.toAbsolutePath().normalize().relativize(file.toAbsolutePath().normalize());
		List<String> parts = new ArrayList<>();
		for (Path part : rel)
			parts.add(part.toString());
		if (parts.isEmpty() || parts.get(0).equals(".."))
			return unit(file);
		parts.set(parts.size() - 1, FilenameUtils.removeExtension(parts.get(parts.size() - 1)));
		if (parts.get(parts.size() - 1).equals("__init__"))
			parts.remove(parts.size() - 1);
		if (parts.isEmpty())
			// the __init__.py in the directory of the program
			return unit(file);
		return String.join(".", parts);
	}

	/**
	 * Yields the name of the unit of the module in the given file (see
	 * {@link PyFileParser}).
	 *
	 * @param file the file
	 *
	 * @return the name of the unit
	 */
	static String unit(
			Path file) {
		return FilenameUtils.removeExtension(file.toString());
	}
}
