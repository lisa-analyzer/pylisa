package it.unive.pylisa.frontend;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.logging.IterationLogger;
import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.Eval_inputContext;
import it.unive.pylisa.antlr.Python3Parser.File_inputContext;
import it.unive.pylisa.antlr.Python3Parser.Single_inputContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.*;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import java.io.IOException;
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
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.lang3.tuple.Triple;

public class PyFrontend extends PyDefinitionVisitorBase {

	/**
	 * Builds an instance of @PyFrontend for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath file path to a Python program
	 * @param notebook whether or not {@code filePath} points to a Jupyter
	 *                     notebook file
	 */
	public PyFrontend(
			String filePath,
			boolean notebook) {
		super(filePath, notebook);
	}

	/**
	 * Builds an instance of @PyFrontend for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder sequence of the indexes of cells of a Jupyter notebook
	 *                      in the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	/**
	 * Builds an instance of @PyFrontend for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath  file path to a Python program
	 * @param notebook  whether or not {@code filePath} points to a Jupyter
	 *                      notebook file
	 * @param cellOrder list of the indexes of cells of a Jupyter notebook in
	 *                      the order they are to be executed. Only valid if
	 *                      {@code notebook} is {@code true}.
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	/**
	 * Builds an instance of @PyFrontend for a given Python program given at the
	 * location filePath, with an explicit source root for module resolution.
	 *
	 * @param filePath   file path to a Python program
	 * @param notebook   whether or not {@code filePath} points to a Jupyter
	 *                       notebook file
	 * @param sourceRoot explicit root directory for resolving project-relative
	 *                       imports; if {@code null}, defaults to the parent of
	 *                       {@code filePath}
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			String sourceRoot) {
		super(filePath, notebook, Collections.emptyList(), sourceRoot);
	}

	/**
	 * Returns the collection of @CFG in a Python program at filePath.
	 *
	 * @return collection of @CFG in file
	 *
	 * @throws IOException            if {@code stream} to file cannot be read
	 *                                    from or closed
	 * @throws AnalysisSetupException if something goes wrong while setting up
	 *                                    the program
	 */
	public Program toLiSAProgram(
			boolean clearClassType)
			throws IOException,
			AnalysisSetupException {
		if (clearClassType) {
			PyClassType.clearAll();
			PyFunctionType.clearAll();
			PyModuleType.clearAll();
			// Re-register __main__ which was set up in the constructor
			PyModuleType.register(currentModule.getName(), currentModule);
		}

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
		objectUnit = (it.unive.lisa.program.CompilationUnit) program.getUnit("builtins.object");
		if (objectUnit == null)
			log.warn("Could not resolve 'builtins.object' after library loading; "
					+ "classes without explicit parents will have no ancestor");
		log.info("Reading file... " + filePath);

		importManager.setProjectLoader(this::loadProjectModuleFile);

		String source;
		Python3Lexer lexer;
		try {
			source = new SourceReader(filePath, notebook, cellOrder).readNormalizedSource();
			lexer = new Python3Lexer(CharStreams.fromString(source, filePath));
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + filePath + "'", e);
		}

		ParseTree tree = parseFileInputStrict(lexer, filePath, source);

		visit(tree);

		PyClassType.all().forEach(types::registerType);
		ModuleUnit pyProgramUnit = new ModuleUnit(
				new SourceCodeLocation("__lisa__", 0, 0),
				program,
				"$PythonProgram");
		program.addUnit(pyProgramUnit);
		CodeMemberDescriptor runDesc = new CodeMemberDescriptor(new SourceCodeLocation("__lisa__", 0, 0), pyProgramUnit,
				false, "run");
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

	public Program toLiSAProgram() throws IOException, AnalysisSetupException {
		return toLiSAProgram(true);
	}

	public ModuleUnit loadProjectModuleFile(
			String moduleName,
			String filePath)
			throws IOException {
		// Save current state
		ModuleUnit savedModule = currentModule;
		Unit savedUnit = currentUnit;
		PyCFG savedCFG = currentCFG;
		Map<String, String> savedImports = imports;
		Collection<ControlFlowStructure> savedCfs = cfs;
		boolean savedShouldPrependUnitAccess = shouldPrependUnitAccess;
		boolean savedFileIsPackage = currentFileIsPackage;
		currentFileIsPackage = filePath.endsWith("__init__.py");

		// The ModuleUnit was already registered by loadProjectModule before
		// calling us
		ModuleUnit newModule = (ModuleUnit) PyModuleType.lookup(moduleName).getUnit();
		currentModule = newModule;
		currentUnit = newModule;
		imports = new HashMap<>();
		cfs = new HashSet<>();

		boolean savedContinue = continueOnUnsupportedStatement;
		continueOnUnsupportedStatement = true; // resilient for sub-module loads
		try {
			String source = SourceReader.readNormalizedFile(filePath);
			Python3Lexer lexer = new Python3Lexer(CharStreams.fromString(source, filePath));
			visitFile_input(parseFileInputStrict(lexer, filePath, source));
		} catch (IOException | RuntimeException e) {
			// Finalize the partially-built $init CFG so it passes LiSA
			// validation
			addRetNodesToCurrentCFG();
			throw e;
		} finally {
			continueOnUnsupportedStatement = savedContinue;
			// Always restore state, even if parsing fails
			currentModule = savedModule;
			currentUnit = savedUnit;
			currentCFG = savedCFG;
			imports = savedImports;
			cfs = savedCfs;
			shouldPrependUnitAccess = savedShouldPrependUnitAccess;
			currentFileIsPackage = savedFileIsPackage;
		}

		return newModule;
	}

	@Override
	public Object visit(
			ParseTree tree) {

		if (tree instanceof File_inputContext)
			return visitFile_input((File_inputContext) tree);

		return null;
	}

	@Override
	public Object visitSingle_input(
			Single_inputContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public PyCFG visitFile_input(
			File_inputContext ctx) {

		currentCFG = new PyCFG(buildInitModuleCFGDescriptor(getLocation(ctx)));
		ImportModule builtinsModule = new ImportModule(currentCFG, getLocation(ctx), "builtins",
				PyModuleType.lookup("builtins").getUnit());
		currentCFG.addNode(builtinsModule, true);
		cfs = new HashSet<>();
		currentModule.addCodeMember(currentCFG);
		Expression targetName = new PythonScopedAttributeAccessRef(this.currentCFG, getLocation(ctx), currentModule,
				new Global(getLocation(ctx), currentModule, "__name__", false));

		PyAssign nameAssign = new PyAssign(this.currentCFG, getLocation(ctx), targetName,
				new StringLiteral(this.currentCFG, getLocation(ctx), currentModule.getName()));
		currentCFG.addNode(nameAssign);
		currentCFG.addEdge(new SequentialEdge(builtinsModule, nameAssign));
		Statement lastStmt = nameAssign;
		for (StmtContext stmt : IterationLogger.iterate(log, ctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Object visited;
			try {
				if (stmt.compound_stmt() != null)
					visited = visitCompound_stmt(stmt.compound_stmt());
				else
					visited = visitSimple_stmt(stmt.simple_stmt());
			} catch (RuntimeException e) {
				if (!continueOnUnsupportedStatement)
					throw e;
				log.warn("[PyLiSA] Skipping unsupported statement at "
						+ getLocation(stmt) + ": " + e.getClass().getSimpleName()
						+ ": " + e.getMessage());
				continue;
			}

			if (!(visited instanceof Triple<?, ?, ?>))
				continue;

			@SuppressWarnings("unchecked")
			Triple<Statement, NodeList<CFG, Statement, Edge>,
					Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) visited;
			currentCFG.getNodeList().mergeWith(st.getMiddle());
			if (lastStmt == null)
				// this is the first instruction
				currentCFG.getEntrypoints().add(st.getLeft());
			else if (!lastStmt.stopsExecution())
				currentCFG.addEdge(new SequentialEdge(lastStmt, st.getLeft()));
			lastStmt = st.getRight();
		}

		addRetNodesToCurrentCFG();
		cfs.forEach(currentCFG.getDescriptor()::addControlFlowStructure);
		currentCFG.simplify();
		return currentCFG;
	}

	@Override
	public Object visitEval_input(
			Eval_inputContext ctx) {
		throw new UnsupportedStatementException();
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
