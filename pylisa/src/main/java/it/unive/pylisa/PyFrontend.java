package it.unive.pylisa;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;
import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.logging.IterationLogger;
import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
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
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.lang3.tuple.Triple;

public class PyFrontend extends PyDefinitionVisitorBase {

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
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
	 * Builds an instance of @PyToCFG for a given Python program given at the
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
	 * Builds an instance of @PyToCFG for a given Python program given at the
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
		if (clearClassType)
			PyClassType.clearAll();

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
		log.info("Reading file... " + filePath);

		Python3Lexer lexer = null;
		try (InputStream stream = mkStream();) {
			lexer = new Python3Lexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + filePath + "'", e);
		}

		Python3Parser parser = new Python3Parser(new CommonTokenStream(lexer));
		ParseTree tree = parser.file_input();

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

		currentCFG = new PyCFG(buildInitModuleCFGDesriptor(getLocation(ctx)));
		ImportModule builtinsModule = new ImportModule(currentCFG, getLocation(ctx), "builtins",
				PyModuleType.lookup("builtins").getUnit());
		Expression target = new PythonUnitAttributeAccessRef(this.currentCFG, getLocation(ctx), currentModule,
				new Global(getLocation(ctx), currentModule, "__builtins__", false));
		PyAssign builtInAssign = new PyAssign(this.currentCFG, getLocation(ctx), target, builtinsModule);
		currentCFG.addNode(builtInAssign, true);
		cfs = new HashSet<>();
		currentModule.addCodeMember(currentCFG);
		Expression targetName = new PythonUnitAttributeAccessRef(this.currentCFG, getLocation(ctx), currentModule,
				new Global(getLocation(ctx), currentModule, "__name__", false));

		PyAssign nameAssign = new PyAssign(this.currentCFG, getLocation(ctx), targetName,
				new StringLiteral(this.currentCFG, getLocation(ctx), currentModule.getName()));
		currentCFG.addNode(nameAssign);
		currentCFG.addEdge(new SequentialEdge(builtInAssign, nameAssign));
		Statement last_stmt = nameAssign;
		for (StmtContext stmt : IterationLogger.iterate(log, ctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Object visited;
			if (stmt.compound_stmt() != null)
				visited = visitCompound_stmt(stmt.compound_stmt());
			else
				visited = visitSimple_stmt(stmt.simple_stmt());

			if (!(visited instanceof Triple<?, ?, ?>))
				continue;

			if (visited != null) {
				@SuppressWarnings("unchecked")
				Triple<Statement, NodeList<CFG, Statement, Edge>,
						Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) visited;
				currentCFG.getNodeList().mergeWith(st.getMiddle());
				if (last_stmt == null)
					// this is the first instruction
					currentCFG.getEntrypoints().add(st.getLeft());
				else
					currentCFG.addEdge(new SequentialEdge(last_stmt, st.getLeft()));
				last_stmt = st.getRight();
			}
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

	private InputStream mkStream() throws FileNotFoundException {
		if (!this.notebook)
			return new FileInputStream(getFilePath());

		Gson gson = new Gson();
		JsonReader reader = gson.newJsonReader(new FileReader(filePath));
		Map<?, ?> map = gson.fromJson(reader, Map.class);
		@SuppressWarnings("unchecked")
		List<Map<?, ?>> cells = (ArrayList<Map<?, ?>>) map.get("cells");
		SortedMap<Integer, String> codeBlocks = new TreeMap<>();
		for (int i = 0; i < cells.size(); i++) {
			Map<?, ?> cell = cells.get(i);
			String ctype = (String) cell.get("cell_type");
			if (ctype.equals("code")) {
				@SuppressWarnings("unchecked")
				List<String> code_list = (List<String>) cell.get("source");
				codeBlocks.put(i, transformToCode(code_list));
			}
		}

		StringBuilder code = new StringBuilder();

		if (cellOrder.isEmpty())
			for (Entry<Integer, String> c : codeBlocks.entrySet())
				code.append(c.getValue()).append("\n");
		else {
			log.warn("The following cells contain code and can be analyzed: " + codeBlocks.keySet());
			for (int idx : cellOrder) {
				String str = codeBlocks.get(idx);
				if (str == null)
					log.warn("Cell " + idx + " does not contain code and will be skipped");
				else
					code.append(str).append("\n");
			}
		}

		return new ByteArrayInputStream(code.toString().getBytes());
	}
}
