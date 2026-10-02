package it.unive.pylisa.frontend;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Objects;
import java.util.Optional;

/**
 * A source of Python modules for imports that are neither files of the
 * analysed project nor library specifications, such as modules generated on
 * demand or found in additional directories (as {@code PYTHONPATH} entries
 * are). A provided module is parsed and analysed like a project file.
 */
@FunctionalInterface
public interface ModuleProvider {

	/**
	 * Yields the source file of a module, if this provider can produce it.
	 *
	 * @param moduleName the dotted name of the module, such as
	 *                       {@code std_msgs.msg}
	 *
	 * @return the Python file defining the module, or empty if this provider
	 *             does not know the module
	 *
	 * @throws IOException if the provider knows the module but cannot produce
	 *                         its file
	 */
	Optional<Path> provide(
			String moduleName)
			throws IOException;

	/**
	 * Yields the provider that looks modules up in a directory, as Python does
	 * for an entry of its module search path: {@code a.b} is
	 * {@code <root>/a/b.py} or {@code <root>/a/b/__init__.py}.
	 *
	 * @param root the directory
	 *
	 * @return the provider
	 */
	static ModuleProvider searchPath(
			Path root) {
		Objects.requireNonNull(root);
		return moduleName -> {
			Path base = root.resolve(moduleName.replace('.', '/'));
			Path module = base.resolveSibling(base.getFileName() + ".py");
			if (Files.isRegularFile(module))
				return Optional.of(module);
			Path pkg = base.resolve("__init__.py");
			if (Files.isRegularFile(pkg))
				return Optional.of(pkg);
			return Optional.empty();
		};
	}
}
