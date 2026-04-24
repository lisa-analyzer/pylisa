package it.unive.pylisa.frontend;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.File_inputContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.definition.DefinitionVisitor;
import it.unive.pylisa.frontend.expression.ExpressionVisitor;
import it.unive.pylisa.frontend.statement.StatementVisitor;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import org.antlr.v4.runtime.BailErrorStrategy;
import org.antlr.v4.runtime.BaseErrorListener;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.Recognizer;
import org.antlr.v4.runtime.misc.ParseCancellationException;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Public entry point to the PyLiSA front-end: parses a Python source file (or
 * Jupyter notebook) into a LiSA {@link Program}.
 * <p>
 * After Chunk 2 this class is an orchestrator: it owns the shared
 * {@link ParserContext}, the stateless {@link ParserSupport} helper, and the
 * three sibling category visitors (expression / statement / definition) that
 * collectively replace the old four-level inheritance chain.
 */
public final class PyFrontend {

	private static final Logger LOG = LogManager.getLogger(PyFrontend.class);

	private final ParserContext ctx;
	private final ParserSupport support;
	private final ExpressionVisitor expr;
	private final StatementVisitor stmt;
	private final DefinitionVisitor def;

	private final String filePath;
	private final boolean notebook;
	private final List<Integer> cellOrder;

	private CFG init;

	public PyFrontend(
			String filePath,
			boolean notebook) {
		this(filePath, notebook, Collections.emptyList(), null);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		this(filePath, notebook, List.of(cellOrder), null);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		this(filePath, notebook, cellOrder, null);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			String sourceRoot) {
		this(filePath, notebook, Collections.emptyList(), sourceRoot);
	}

	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot) {
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;

		this.ctx = new ParserContext();
		this.ctx.filePath(filePath);
		this.ctx.currentFileIsPackage(filePath != null && filePath.endsWith("__init__.py"));

		Program program = new Program(new PythonFeatures(), new PythonTypeSystem());
		this.ctx.program(program);
		ModuleUnit currentModule = new ModuleUnit(new SourceCodeLocation(filePath, 0, 0), program, "__main__");
		this.ctx.currentModule(currentModule);
		this.ctx.currentUnit(currentModule);
		this.init = makeInit(program);
		this.ctx.init(this.init);
		Path baseDir = (sourceRoot != null)
				? Path.of(sourceRoot)
				: (filePath != null) ? Path.of(filePath).getParent() : Path.of(".");
		this.ctx.importManager(new PythonModuleImportManager(program, init, baseDir));
		program.addUnit(currentModule);
		PyModuleType.register("__main__", currentModule);

		this.support = new ParserSupport(ctx);
		this.expr = new ExpressionVisitor(ctx, support);
		this.stmt = new StatementVisitor(ctx, support);
		this.def = new DefinitionVisitor(ctx, support);
		this.ctx.wireVisitors(expr, stmt, def);
		LOG.debug("PyFrontend wired for {}", filePath);
	}

	public PyFrontend setContinueOnUnsupportedStatement(
			boolean value) {
		this.ctx.continueOnUnsupportedStatement(value);
		return this;
	}

	public String getFilePath() {
		return filePath;
	}

	public Program toLiSAProgram() throws IOException, AnalysisSetupException {
		return toLiSAProgram(true);
	}

	public Program toLiSAProgram(
			boolean clearClassType)
			throws IOException,
			AnalysisSetupException {
		if (clearClassType) {
			PyClassType.clearAll();
			PyFunctionType.clearAll();
			PyModuleType.clearAll();
			PyModuleType.register(ctx.currentModule().getName(), ctx.currentModule());
		}

		Program program = ctx.program();
		TypeSystem types = program.getTypes();
		types.registerType(PyLambdaType.INSTANCE);
		types.registerType(BoolType.INSTANCE);
		types.registerType(StringType.INSTANCE);
		types.registerType(Int32Type.INSTANCE);
		types.registerType(Float32Type.INSTANCE);
		types.registerType(NullType.INSTANCE);
		types.registerType(VoidType.INSTANCE);
		types.registerType(Untyped.INSTANCE);
		LibrarySpecificationProvider.load(program, init);

		LibrarySpecificationProvider.importPythonModule(program, "builtins", init);
		ctx.objectUnit((CompilationUnit) program.getUnit("builtins.object"));
		if (ctx.objectUnit() == null)
			LOG.warn("Could not resolve 'builtins.object' after library loading; "
					+ "classes without explicit parents will have no ancestor");
		LOG.info("Reading file... {}", filePath);

		ctx.importManager().setProjectLoader(this::loadProjectModuleFile);

		String source;
		Python3Lexer lexer;
		try {
			source = new SourceReader(filePath, notebook, cellOrder).readNormalizedSource();
			lexer = new Python3Lexer(CharStreams.fromString(source, filePath));
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + filePath + "'", e);
		}

		File_inputContext tree = parseFileInputStrict(lexer, filePath, source);

		stmt.visitFile_input(tree);

		PyClassType.all().forEach(types::registerType);
		ModuleUnit pyProgramUnit = new ModuleUnit(
				new SourceCodeLocation("__lisa__", 0, 0),
				program,
				"$PythonProgram");
		program.addUnit(pyProgramUnit);
		CodeMemberDescriptor runDesc = new CodeMemberDescriptor(new SourceCodeLocation("__lisa__", 0, 0),
				pyProgramUnit, false, "run");
		runDesc.setOverridable(false);
		PyCFG runCFG = new PyCFG(runDesc);
		pyProgramUnit.addCodeMember(runCFG);
		ImportModule importModule_builtins = new ImportModule(runCFG, new SourceCodeLocation("__lisa__", 0, 0),
				"builtins", (ModuleUnit) PyModuleType.lookup("builtins").getUnit());
		runCFG.addNode(importModule_builtins, true);
		ImportModule importModuleMain = new ImportModule(runCFG, new SourceCodeLocation("__lisa__", 0, 0), "__main__",
				(ModuleUnit) PyModuleType.lookup("__main__").getUnit());
		runCFG.addNode(importModuleMain);
		runCFG.addEdge(new SequentialEdge(importModule_builtins, importModuleMain));
		Ret ret = new Ret(runCFG, SyntheticLocation.INSTANCE);
		runCFG.addNode(ret);
		runCFG.addEdge(new SequentialEdge(importModuleMain, ret));
		program.addEntryPoint(runCFG);
		return program;
	}

	public ModuleUnit loadProjectModuleFile(
			String moduleName,
			String modulePath)
			throws IOException {
		ModuleUnit savedModule = ctx.currentModule();
		Unit savedUnit = ctx.currentUnit();
		PyCFG savedCFG = ctx.currentCFG();
		Map<String, String> savedImports = ctx.imports();
		Collection<ControlFlowStructure> savedCfs = ctx.cfs();
		boolean savedShouldPrependUnitAccess = ctx.shouldPrependUnitAccess();
		boolean savedFileIsPackage = ctx.currentFileIsPackage();
		ctx.currentFileIsPackage(modulePath.endsWith("__init__.py"));

		// The ModuleUnit was already registered by loadProjectModule before
		// calling us
		ModuleUnit newModule = (ModuleUnit) PyModuleType.lookup(moduleName).getUnit();
		ctx.currentModule(newModule);
		ctx.currentUnit(newModule);
		ctx.imports(new HashMap<>());
		ctx.cfs(new HashSet<>());

		boolean savedContinue = ctx.continueOnUnsupportedStatement();
		ctx.continueOnUnsupportedStatement(true);
		try {
			String source = SourceReader.readNormalizedFile(modulePath);
			Python3Lexer lexer = new Python3Lexer(CharStreams.fromString(source, modulePath));
			stmt.visitFile_input(parseFileInputStrict(lexer, modulePath, source));
		} catch (IOException | RuntimeException e) {
			// Finalize the partially-built $init CFG so it passes LiSA
			// validation
			support.addRetNodesToCurrentCFG();
			throw e;
		} finally {
			ctx.continueOnUnsupportedStatement(savedContinue);
			ctx.currentModule(savedModule);
			ctx.currentUnit(savedUnit);
			ctx.currentCFG(savedCFG);
			ctx.imports(savedImports);
			ctx.cfs(savedCfs);
			ctx.shouldPrependUnitAccess(savedShouldPrependUnitAccess);
			ctx.currentFileIsPackage(savedFileIsPackage);
		}

		return newModule;
	}

	private CFG makeInit(
			Program program) {
		CFG init = new CFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, program, false, "LiSA$init"));
		init.addNode(new Ret(init, SyntheticLocation.INSTANCE), true);
		program.addCodeMember(init);
		return init;
	}

	private File_inputContext parseFileInputStrict(
			Python3Lexer lexer,
			String sourceName,
			String source)
			throws IOException {
		BaseErrorListener strictErrors = new BaseErrorListener() {
			@Override
			public void syntaxError(
					Recognizer<?, ?> recognizer,
					Object offendingSymbol,
					int line,
					int charPositionInLine,
					String msg,
					RecognitionException e) {
				throw new ParseCancellationException(SyntaxErrorFormatter.format(sourceName, source, offendingSymbol,
						line, charPositionInLine, msg));
			}
		};

		lexer.removeErrorListeners();
		lexer.addErrorListener(strictErrors);

		Python3Parser parser = new Python3Parser(new CommonTokenStream(lexer));
		parser.removeErrorListeners();
		parser.addErrorListener(strictErrors);
		parser.setErrorHandler(new BailErrorStrategy());

		try {
			return parser.file_input();
		} catch (ParseCancellationException ex) {
			String detail = ex.getMessage();
			if (detail == null && ex.getCause() instanceof RecognitionException) {
				RecognitionException re = (RecognitionException) ex.getCause();
				org.antlr.v4.runtime.Token tok = re.getOffendingToken();
				if (tok != null) {
					detail = SyntaxErrorFormatter.format(
							sourceName,
							source,
							tok,
							tok.getLine(),
							tok.getCharPositionInLine(),
							"unexpected token '" + tok.getText() + "'");
				}
			}
			throw new IOException("Invalid Python input in '" + sourceName + "': " + detail, ex);
		}
	}
}
