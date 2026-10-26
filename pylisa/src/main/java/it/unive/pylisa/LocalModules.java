package it.unive.pylisa;

import it.unive.pylisa.antlr.PythonLexer;
import it.unive.pylisa.antlr.PythonParser;
import it.unive.pylisa.antlr.PythonParser.AssignmentContext;
import it.unive.pylisa.antlr.PythonParser.Compound_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_nameContext;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParser.Function_defContext;
import it.unive.pylisa.antlr.PythonParser.Function_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.Import_fromContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_nameContext;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.scoping.PyScope;
import it.unive.pylisa.scoping.Symtable;
import it.unive.pylisa.scoping.SymtableBuilder;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
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
		Set<Path> visiting = new HashSet<>();
		visiting.add(file.toAbsolutePath().normalize());
		return of(tree, file, root, visiting);
	}

	// the imports of the file, whose members are resolved to the modules
	// defining them (see aliases), where visiting holds the files whose
	// aliases are being computed
	private static Imports of(
			ParseTree tree,
			Path file,
			Path root,
			Set<Path> visiting) {
		Map<String, String> modules = new LinkedHashMap<>();
		Map<String, LocalImports.Member> members = new LinkedHashMap<>();
		Map<ParserRuleContext, Set<String>> stars = new LinkedHashMap<>();
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
				// '...' is a single token
				int level = from.DOT().size() + 3 * from.ELLIPSIS().size();
				String name = from.dotted_name() == null ? "" : String.join(".", parts(from.dotted_name()));
				Path module = resolve(root, file, level, name);
				if (module != null)
					dependencies.add(module);
				if (from.import_from_targets().import_from_as_names() == null) {
					// from m import *: the public names of m, if they are known
					Set<String> imported = module == null ? null : publicNames(module, root);
					if (imported != null) {
						for (String member : imported)
							members.put(member, chase(
									new LocalImports.Member(unit(module), moduleName(module, root), member), root,
									visiting));
						stars.put(from, imported);
					}
					continue;
				}
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
						members.put(local,
								chase(new LocalImports.Member(unit(module), moduleName(module, root), member),
										root, visiting));
				}
			}

		Map<String, String> names = new LinkedHashMap<>();
		for (Path dependency : dependencies)
			names.put(unit(dependency), moduleName(dependency, root));
		// the aliases of the imported modules (see aliases)
		Map<String, Map<String, LocalImports.Member>> reexports = new LinkedHashMap<>();
		Map<String, Map<String, String>> submodules = new LinkedHashMap<>();
		// also the modules bound by the imported modules (e.g. to_import2 in
		// to_import.to_import2, if to_import imports it)
		List<String> pending = new ArrayList<>(new LinkedHashSet<>(modules.values()));
		while (!pending.isEmpty()) {
			String unit = pending.remove(0);
			if (reexports.containsKey(unit))
				continue;
			Aliases aliases = aliases(Path.of(unit + ".py"), root, visiting);
			reexports.put(unit, aliases.members());
			submodules.put(unit, aliases.modules());
			aliases.names().forEach(names::putIfAbsent);
			pending.addAll(aliases.modules().values());
		}
		return new Imports(new LocalImports(modules, members, names, stars, reexports, submodules), dependencies);
	}

	// the member of the module defining it: the one the given member is an
	// alias of (see aliases), if any
	private static LocalImports.Member chase(
			LocalImports.Member member,
			Path root,
			Set<Path> visiting) {
		LocalImports.Member alias = aliases(Path.of(member.unit() + ".py"), root, visiting).members()
				.get(member.name());
		return alias == null ? member : alias;
	}

	/**
	 * The names of a module that are aliases (see {@link #aliases}).
	 *
	 * @param members the names that are aliases of members of modules, resolved
	 *                    to the modules defining them
	 * @param modules the names bound to modules (e.g. {@code m} after
	 *                    {@code import m}), mapped to the units of the modules
	 * @param names   the (dotted) names of these modules, indexed by their
	 *                    units
	 */
	record Aliases(
			Map<String, LocalImports.Member> members,
			Map<String, String> modules,
			Map<String, String> names) {

		static final Aliases NONE = new Aliases(Map.of(), Map.of(), Map.of());
	}

	/**
	 * Yields the names of the given module that are aliases of members of other
	 * modules (or of its own functions), resolved to the modules defining them:
	 * the names bound once in the namespace of the module, by a statement at
	 * its top level (not inside a compound statement, that might not execute
	 * it) that is {@code from m import y [as x]}, or an assignment
	 * {@code x = m.y} where {@code m} is an imported module, or {@code x = y}
	 * where {@code y} is a member imported from another module, or a function
	 * of the module defined once (without decorators) at its top level. The
	 * names bound once in the same way by an import of a module (e.g.
	 * {@code import m}, or {@code from . import m}) are aliases of the module.
	 * A module whose globals might be bound by its functions, or by a
	 * {@code from ... import *}, has no aliases. The aliases of a module that
	 * is being visited (an import cycle) are not resolved.
	 *
	 * @param module   the file of the module
	 * @param root     the directory of the analyzed program
	 * @param visiting the files whose aliases are being computed
	 *
	 * @return the aliases
	 */
	private static Aliases aliases(
			Path module,
			Path root,
			Set<Path> visiting) {
		Path key = module.toAbsolutePath().normalize();
		if (!Files.isRegularFile(module) || !visiting.add(key))
			return Aliases.NONE;
		try {
			File_inputContext tree;
			try {
				PythonLexer lexer = new PythonLexer(CharStreams.fromPath(module, StandardCharsets.UTF_8));
				tree = new PythonParser(new CommonTokenStream(lexer)).file_input();
			} catch (IOException e) {
				// the module is parsed again later, where the error is reported
				return Aliases.NONE;
			}
			Symtable symtable = SymtableBuilder.build(module.toString(), moduleName(module, root), tree);
			PyScope scope = symtable.getModuleScope();
			if (scope.hasStarImport() || !symtable.globalsBoundOnlyByModule())
				return Aliases.NONE;
			LocalImports imports = of(tree, module, root, visiting).bindings();
			Map<String, LocalImports.Member> members = new LinkedHashMap<>();
			Map<String, String> modules = new LinkedHashMap<>(), names = new LinkedHashMap<>();
			for (String name : scope.getSymbols().keySet()) {
				List<ParserRuleContext> bindings = symtable.bindingsOf(scope, name);
				if (bindings.size() != 1 || !topLevel(bindings.get(0)))
					continue;
				if (imports.module(name) != null && (ancestor(bindings.get(0), Import_nameContext.class) != null
						|| ancestor(bindings.get(0), Import_fromContext.class) != null)) {
					// import m, or from . import m
					String unit = imports.module(name);
					modules.put(name, unit);
					names.put(unit, imports.moduleName(unit));
					continue;
				}
				LocalImports.Member alias = alias(bindings.get(0), name, module, root, imports, symtable, visiting);
				if (alias != null)
					members.put(name, alias);
			}
			return new Aliases(members, modules, names);
		} finally {
			visiting.remove(key);
		}
	}

	// the member that the binding of name makes it an alias of, if any (see
	// aliases)
	private static LocalImports.Member alias(
			ParserRuleContext binding,
			String name,
			Path module,
			Path root,
			LocalImports imports,
			Symtable symtable,
			Set<Path> visiting) {
		if (ancestor(binding, Import_fromContext.class) != null)
			// from m import y as name, already resolved (null for a submodule)
			return imports.member(name);
		AssignmentContext assignment = ancestor(binding, AssignmentContext.class);
		if (assignment == null || assignment.star_targets().size() != 1
				|| !assignment.star_targets(0).getText().equals(name) || assignment.annotated_rhs() == null
				|| assignment.annotated_rhs().star_expressions() == null)
			return null;
		String value = assignment.annotated_rhs().getText();
		if (!value.matches("[A-Za-z_]\\w*(\\.[A-Za-z_]\\w*)*"))
			return null;
		int dot = value.lastIndexOf('.');
		if (dot >= 0) {
			// name = m.y, with m an imported module (and y not a submodule)
			String prefix = value.substring(0, dot), member = value.substring(dot + 1);
			String unit = imports.module(prefix);
			if (unit == null || imports.module(value) != null)
				return null;
			return chase(new LocalImports.Member(unit, imports.moduleName(unit), member), root, visiting);
		}
		if (imports.member(value) != null)
			// name = y, with y imported from another module
			return imports.member(value);
		// name = f, with f a function of the module defined once
		List<ParserRuleContext> definitions = symtable.bindingsOf(symtable.getModuleScope(), value);
		if (definitions.size() != 1 || !(definitions.get(0).getParent() instanceof Function_def_rawContext))
			return null;
		Function_defContext def = ancestor(definitions.get(0), Function_defContext.class);
		Compound_stmtContext statement = ancestor(def, Compound_stmtContext.class);
		if (def.decorators() != null || statement == null || !topLevel(statement))
			return null;
		return new LocalImports.Member(unit(module), moduleName(module, root), value);
	}

	// whether the node is in a statement at the top level of its module, that
	// is not inside a compound statement
	private static boolean topLevel(
			ParserRuleContext node) {
		for (ParseTree p = node.getParent(); p != null; p = p.getParent())
			if (p instanceof Compound_stmtContext)
				return false;
			else if (p instanceof File_inputContext)
				return true;
		return false;
	}

	// the closest ancestor of the node of the given class, if any
	private static <T extends ParserRuleContext> T ancestor(
			ParseTree node,
			Class<T> cls) {
		for (ParseTree p = node.getParent(); p != null; p = p.getParent())
			if (cls.isInstance(p))
				return cls.cast(p);
		return null;
	}

	/**
	 * Yields the names bound by {@code from m import *}, for the given module
	 * {@code m}: the names bound in the namespace of the module (by its code,
	 * or by its functions through {@code global} declarations) that do not
	 * start with an underscore. These are not known if the module defines
	 * {@code __all__} (the names it lists are bound instead), or if it contains
	 * a {@code from ... import *} itself. As the code of the module might not
	 * bind all of them (e.g. names bound inside an {@code if}), they might be
	 * unbound after the import (see {@code FromImport}).
	 *
	 * @param module the file of the module
	 * @param root   the directory of the analyzed program
	 *
	 * @return the names, or {@code null} if they are not known
	 */
	static Set<String> publicNames(
			Path module,
			Path root) {
		File_inputContext tree;
		try {
			PythonLexer lexer = new PythonLexer(CharStreams.fromPath(module, StandardCharsets.UTF_8));
			tree = new PythonParser(new CommonTokenStream(lexer)).file_input();
		} catch (IOException e) {
			// the module is parsed again later, where the error is reported
			return null;
		}
		PyScope scope = SymtableBuilder.build(module.toString(), moduleName(module, root), tree).getModuleScope();
		if (scope.hasStarImport())
			return null;
		Set<String> names = new TreeSet<>();
		bound(scope, names);
		if (names.contains("__all__"))
			return null;
		names.removeIf(n -> n.startsWith("_"));
		return names;
	}

	// the names of the module bound by the given scope, or by the ones nested
	// into it
	private static void bound(
			PyScope scope,
			Set<String> names) {
		for (String name : scope.getSymbols().keySet())
			if (scope.binds(name) && scope.kindOf(name).isGlobal())
				names.add(name);
		for (PyScope child : scope.getChildren())
			bound(child, names);
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
