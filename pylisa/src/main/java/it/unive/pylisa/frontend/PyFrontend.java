package it.unive.pylisa.frontend;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.definition.DefinitionVisitor;
import it.unive.pylisa.frontend.expression.ExpressionVisitor;
import it.unive.pylisa.frontend.statement.StatementVisitor;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.ProgramSettings;
import it.unive.pylisa.program.PyProgram;
import it.unive.pylisa.program.PySyntheticLocation;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Public entry point to the PyLiSA front-end: parses a Python source file (or
 * Jupyter notebook) into a LiSA {@link Program}.
 * <p>
 * Post-Chunk-7 this class is a thin orchestrator. Heavy-lifting is delegated to
 * {@link AntlrPipeline} (lexer + parser), {@link ProgramFinalizer} (type
 * registration + entry-point CFG), and {@link ModuleLoaderCallback} (recursive
 * sub-module parsing). The shared {@link ParserContext} holds mutable parse
 * state; {@link ParserSupport} holds stateless helpers; the three category
 * visitors (expression / statement / definition) translate Python AST into LiSA
 * IR.
 */
public final class PyFrontend {

	private static final Logger LOG = LogManager.getLogger(PyFrontend.class);

	private final ParserContext ctx;
	private final ParserSupport support;
	private final ExpressionVisitor expr;
	private final StatementVisitor stmt;
	private final DefinitionVisitor def;
	private final ModuleLoaderCallback moduleLoader;

	private final String filePath;
	private final boolean notebook;
	private final List<Integer> cellOrder;
	private final boolean strict;

	private final CFG init;

	public PyFrontend(
			String filePath,
			boolean notebook) {
		this(filePath, notebook, Collections.emptyList(), null, false);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		this(filePath, notebook, List.of(cellOrder), null, false);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		this(filePath, notebook, cellOrder, null, false);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			String sourceRoot) {
		this(filePath, notebook, Collections.emptyList(), sourceRoot, false);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot) {
		this(filePath, notebook, cellOrder, sourceRoot, false);
	}

	/**
	 * Returns a {@link PyFrontend} in strict mode: any {@code UNSOUND}
	 * diagnostic emitted during parsing is promoted to a
	 * {@link DiagnosticReporter.StrictModeViolation}. Useful for regression
	 * tests that want to guarantee no silent approximations occur.
	 */
	public static PyFrontend strict(
			String filePath) {
		return new PyFrontend(filePath, false, Collections.emptyList(), null, true);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot,
			boolean strict) {
		this(filePath, notebook, cellOrder, sourceRoot, strict, null);
	}

	/**
	 * Builds a frontend whose program carries no settings. Accepts an
	 * optional {@code entryModuleName} substituting the default
	 * {@code "__main__"} identity of the entry file. The entry {@link ModuleUnit} is created with that name and
	 * registered in {@link PyModuleType} under that name, so a later
	 * import of the same dotted module (e.g. another file doing
	 * {@code import mcpgateway.main}) resolves directly to the existing
	 * unit instead of producing a second compilation-unit identity.
	 * <p>
	 * When {@code entryModuleName} is {@code null}, the legacy behaviour
	 * is preserved (entry module is {@code "__main__"}). The entry file's
	 * absolute path is then seeded into the import manager's path→unit
	 * cache as a fallback so that an in-tree qualified-name import still
	 * deduplicates to the same unit; this mirrors how real Python module
	 * caching ({@code sys.modules}) prevents double execution of the
	 * served file under uvicorn-style deployments.
	 *
	 * @param entryModuleName the dotted module name to assign the entry
	 *                            file (e.g. {@code "mcpgateway.main"}),
	 *                            or {@code null} for the default
	 *                            {@code "__main__"}
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot,
			boolean strict,
			String entryModuleName) {
		this(filePath, notebook, cellOrder, sourceRoot, strict, entryModuleName, ProgramSettings.NONE);
	}

	/**
	 * Builds a frontend for a Python file whose program carries the given
	 * settings of the environment it is analysed in, which library models
	 * read.
	 *
	 * @param filePath the path of the Python entry file
	 * @param settings the settings the translated program carries
	 */
	public PyFrontend(
			String filePath,
			ProgramSettings settings) {
		this(filePath, false, Collections.emptyList(), null, false, null, settings);
	}

	/**
	 * Master constructor: see
	 * {@link #PyFrontend(String, boolean, List, String, boolean, String)};
	 * the translated program also carries the given settings.
	 *
	 * @param filePath        the path of the entry file
	 * @param notebook        whether the entry file is a Jupyter notebook
	 * @param cellOrder       the order in which the cells of a notebook
	 *                            run, or empty for their order in the file
	 * @param sourceRoot      the root of the project's sources, or
	 *                            {@code null} for the entry file's directory
	 * @param strict          whether unsound translations fail the
	 *                            translation
	 * @param entryModuleName the dotted module name of the entry file, or
	 *                            {@code null} for {@code "__main__"}
	 * @param settings        the settings of the environment the program
	 *                            is analysed in, which library models read
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot,
			boolean strict,
			String entryModuleName,
			ProgramSettings settings) {
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
		this.strict = strict;

		this.ctx = new ParserContext();
		this.ctx.reporter(new DiagnosticReporter(strict));
		this.ctx.filePath(filePath);
		this.ctx.currentFileIsPackage(filePath != null && filePath.endsWith("__init__.py"));

		Program program = new PyProgram(settings);
		this.ctx.program(program);
		String mainName = (entryModuleName != null && !entryModuleName.isBlank())
				? entryModuleName
				: "__main__";
		ModuleUnit mainModule = new ModuleUnit(new SourceCodeLocation(filePath, 0, 0), program, mainName);
		this.ctx.currentModule(mainModule);
		this.ctx.currentUnit(mainModule);
		this.init = makeInit(program);
		this.ctx.init(this.init);
		Path baseDir = (sourceRoot != null)
				? Path.of(sourceRoot)
				: (filePath != null) ? Path.of(filePath).getParent() : Path.of(".");
		this.ctx.importManager(new PythonModuleImportManager(program, init, baseDir));
		program.addUnit(mainModule);
		PyModuleType.register(mainName, mainModule);
		// When the caller supplied an explicit entryModuleName (e.g.
		// "mcpgateway.main"), seed the import-manager's path → unit cache
		// with the entry file so that a later qualified-name import (from
		// another module doing `from mcpgateway.main import …`) resolves
		// to the existing entry unit instead of re-parsing it under a
		// second compilation-unit identity. This matches the real-Python
		// uvicorn deployment, where `mcpgateway/main.py` is loaded once
		// as `mcpgateway.main` and any subsequent `import mcpgateway.main`
		// returns the cached object via {@code sys.modules}.
		// <p>
		// Without an explicit entryModuleName we leave the bug visible on
		// purpose: at runtime, `python mcpgateway/main.py` and a
		// concurrent `import mcpgateway.main` from elsewhere ARE two
		// distinct Python modules (sys.modules keys by name, not by
		// file), and pylisa's pre-existing behaviour matches that
		// semantics. Opting into the uvicorn-served model is the user's
		// signal via --entry-module.
		if (entryModuleName != null && !entryModuleName.isBlank() && filePath != null) {
			this.ctx.importManager().registerEntryFile(
					Path.of(filePath), mainModule);
		}

		this.support = new ParserSupport(ctx);
		this.expr = new ExpressionVisitor(ctx, support);
		this.stmt = new StatementVisitor(ctx, support);
		this.def = new DefinitionVisitor(ctx, support);
		this.ctx.wireVisitors(expr, stmt, def);
		this.moduleLoader = new ModuleLoaderCallback(ctx, support, stmt);
		LOG.debug("PyFrontend wired for {} (entry module: {})", filePath, mainName);
	}

	public PyFrontend setContinueOnUnsupportedStatement(
			boolean value) {
		this.ctx.continueOnUnsupportedStatement(value);
		return this;
	}

	/**
	 * Registers a provider of modules that are neither files of the analysed
	 * project nor library specifications. Must be called before
	 * {@link #toLiSAProgram()}.
	 *
	 * @param provider the provider
	 *
	 * @return this frontend
	 */
	public PyFrontend addModuleProvider(
			ModuleProvider provider) {
		this.ctx.importManager().addModuleProvider(provider);
		return this;
	}

	/**
	 * Adds a directory to the module search path, as an entry of
	 * {@code PYTHONPATH} does. Must be called before {@link #toLiSAProgram()}.
	 *
	 * @param root the directory
	 *
	 * @return this frontend
	 */
	public PyFrontend addModuleSearchPath(
			Path root) {
		return addModuleProvider(ModuleProvider.searchPath(root));
	}

	/**
	 * Marks the given fully-qualified module names as excluded from parsing
	 * and analysis. Imports referencing them resolve to
	 * {@code UnknownModuleUnit} stubs; calls into them return top under the
	 * existing unknown-module semantics. Useful for taming analyzer-hostile
	 * leaves whose caller fan-in dominates fixpoint cost without
	 * contributing to the property being analyzed (e.g. routing).
	 *
	 * @param excluded the dotted module names to skip; may be {@code null}
	 *                     or empty
	 * @return this frontend, for chaining
	 */
	public PyFrontend setExcludedModules(
			java.util.Set<String> excluded) {
		this.ctx.importManager().setExcludedModules(excluded);
		return this;
	}

	public String getFilePath() {
		return filePath;
	}

	/**
	 * Returns the diagnostic reporter associated with this frontend instance.
	 * Tests and tooling can inspect {@link DiagnosticReporter#events()} after
	 * {@link #toLiSAProgram(boolean)} completes to see what the parse produced.
	 */
	public DiagnosticReporter reporter() {
		return ctx.reporter();
	}

	/**
	 * Returns {@code true} iff this frontend was constructed in strict mode.
	 */
	public boolean strict() {
		return strict;
	}

	public Program toLiSAProgram() throws IOException, AnalysisSetupException {
		return toLiSAProgram(true);
	}

	public Program toLiSAProgram(
			boolean clearClassType)
			throws IOException,
			AnalysisSetupException {
		if (clearClassType)
			resetTypeRegistries();

		ProgramFinalizer finalizer = new ProgramFinalizer(ctx, init);
		finalizer.prepare();

		LOG.info("reading {}", filePath);
		ctx.importManager().setProjectLoader(moduleLoader::load);

		String source;
		try {
			source = new SourceReader(filePath, notebook, cellOrder).readNormalizedSource();
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + filePath + "'", e);
		}
		stmt.visitFile_input(new AntlrPipeline(filePath, source).parseFile());

		return finalizer.finalizeProgram();
	}

	public ModuleUnit loadProjectModuleFile(
			String moduleName,
			String modulePath)
			throws IOException {
		return moduleLoader.load(moduleName, modulePath);
	}

	private void resetTypeRegistries() {
		PyClassType.clearAll();
		PyFunctionType.clearAll();
		PyModuleType.clearAll();
		PyModuleType.register(ctx.currentModule().getName(), ctx.currentModule());
	}

	private static CFG makeInit(
			Program program) {
		CFG init = new CFG(new CodeMemberDescriptor(PySyntheticLocation.INSTANCE, program, false, "LiSA$init"));
		init.addNode(new Ret(init, PySyntheticLocation.INSTANCE), true);
		program.addCodeMember(init);
		return init;
	}
}
