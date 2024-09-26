package it.unive.pylisa;

import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.stream.Collectors;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;

import it.unive.lisa.AnalysisSetupException;
import it.unive.lisa.logging.IterationLogger;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CodeUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.VariableTableEntry;
import it.unive.lisa.program.cfg.controlFlow.ControlFlowStructure;
import it.unive.lisa.program.cfg.controlFlow.IfThenElse;
import it.unive.lisa.program.cfg.controlFlow.Loop;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.FalseEdge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.edge.TrueEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Return;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.lisa.program.cfg.statement.logic.Not;
import it.unive.lisa.program.cfg.statement.numeric.Division;
import it.unive.lisa.program.cfg.statement.numeric.Subtraction;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.annotationvalues.DecoratedAnnotation;
import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.AddContext;
import it.unive.pylisa.antlr.Python3Parser.And_exprContext;
import it.unive.pylisa.antlr.Python3Parser.And_testContext;
import it.unive.pylisa.antlr.Python3Parser.AnnassignContext;
import it.unive.pylisa.antlr.Python3Parser.ArglistContext;
import it.unive.pylisa.antlr.Python3Parser.ArgumentContext;
import it.unive.pylisa.antlr.Python3Parser.Arith_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Assert_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Async_funcdefContext;
import it.unive.pylisa.antlr.Python3Parser.Async_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.AtomContext;
import it.unive.pylisa.antlr.Python3Parser.Atom_exprContext;
import it.unive.pylisa.antlr.Python3Parser.AugassignContext;
import it.unive.pylisa.antlr.Python3Parser.Break_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.ClassdefContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_forContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_ifContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_iterContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_opContext;
import it.unive.pylisa.antlr.Python3Parser.ComparisonContext;
import it.unive.pylisa.antlr.Python3Parser.Compound_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Continue_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratedContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorContext;
import it.unive.pylisa.antlr.Python3Parser.DecoratorsContext;
import it.unive.pylisa.antlr.Python3Parser.Del_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.antlr.Python3Parser.DivContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_namesContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Encoding_declContext;
import it.unive.pylisa.antlr.Python3Parser.Eval_inputContext;
import it.unive.pylisa.antlr.Python3Parser.Except_clauseContext;
import it.unive.pylisa.antlr.Python3Parser.ExprContext;
import it.unive.pylisa.antlr.Python3Parser.Expr_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.ExprlistContext;
import it.unive.pylisa.antlr.Python3Parser.FactorContext;
import it.unive.pylisa.antlr.Python3Parser.File_inputContext;
import it.unive.pylisa.antlr.Python3Parser.FloorDivContext;
import it.unive.pylisa.antlr.Python3Parser.Flow_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.For_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.FuncdefContext;
import it.unive.pylisa.antlr.Python3Parser.Global_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.If_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_namesContext;
import it.unive.pylisa.antlr.Python3Parser.Import_fromContext;
import it.unive.pylisa.antlr.Python3Parser.Import_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.LambdefContext;
import it.unive.pylisa.antlr.Python3Parser.Lambdef_nocondContext;
import it.unive.pylisa.antlr.Python3Parser.Left_shiftContext;
import it.unive.pylisa.antlr.Python3Parser.Mat_mulContext;
import it.unive.pylisa.antlr.Python3Parser.MinusContext;
import it.unive.pylisa.antlr.Python3Parser.ModContext;
import it.unive.pylisa.antlr.Python3Parser.MulContext;
import it.unive.pylisa.antlr.Python3Parser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Not_testContext;
import it.unive.pylisa.antlr.Python3Parser.Or_testContext;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.Pass_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.PowerContext;
import it.unive.pylisa.antlr.Python3Parser.Raise_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Return_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Right_shiftContext;
import it.unive.pylisa.antlr.Python3Parser.Simple_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Single_inputContext;
import it.unive.pylisa.antlr.Python3Parser.SliceopContext;
import it.unive.pylisa.antlr.Python3Parser.Small_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Star_exprContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.Subscript_Context;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.SuiteContext;
import it.unive.pylisa.antlr.Python3Parser.TermContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
import it.unive.pylisa.antlr.Python3Parser.TestOrStarContext;
import it.unive.pylisa.antlr.Python3Parser.Test_nocondContext;
import it.unive.pylisa.antlr.Python3Parser.TestlistContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_compContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_star_exprContext;
import it.unive.pylisa.antlr.Python3Parser.TfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.TrailerContext;
import it.unive.pylisa.antlr.Python3Parser.Try_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargContext;
import it.unive.pylisa.antlr.Python3Parser.TypedargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VarpositionalContext;
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.While_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.With_itemContext;
import it.unive.pylisa.antlr.Python3Parser.With_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Xor_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_stmtContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.Break;
import it.unive.pylisa.cfg.expression.Continue;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Empty;
import it.unive.pylisa.cfg.expression.LambdaExpression;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAccessInstanceGlobal;
import it.unive.pylisa.cfg.expression.PyAddition;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyBitwiseAnd;
import it.unive.pylisa.cfg.expression.PyBitwiseLeftShift;
import it.unive.pylisa.cfg.expression.PyBitwiseNot;
import it.unive.pylisa.cfg.expression.PyBitwiseOr;
import it.unive.pylisa.cfg.expression.PyBitwiseRIghtShift;
import it.unive.pylisa.cfg.expression.PyBitwiseXor;
import it.unive.pylisa.cfg.expression.PyDoubleArrayAccess;
import it.unive.pylisa.cfg.expression.PyFloorDiv;
import it.unive.pylisa.cfg.expression.PyIn;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyMatMul;
import it.unive.pylisa.cfg.expression.PyMultiplication;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PyRemainder;
import it.unive.pylisa.cfg.expression.PySingleArrayAccess;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.cfg.expression.PyTernaryOperator;
import it.unive.pylisa.cfg.expression.PyUnresolvedCall;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.SetCreation;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.FunctionDef;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PyFrontend extends Python3ParserBaseVisitor<Object> {

	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	private static final SequentialEdge SEQUENTIAL_SINGLETON = new SequentialEdge();

	private static final Logger log = LogManager.getLogger(PyFrontend.class);

	private Map<String, String> imports = new HashMap<>();
	/**
	 * Python program file path.
	 */
	private final String filePath;

	/**
	 * The LiSA program obtained from the Python program at filePath.
	 */
	private final Program program;

	/**
	 * The unit currently under parsing
	 */
	private Unit currentUnit;

	/**
	 * Current CFG to parse
	 */
	private PyCFG currentCFG;

	private Collection<ControlFlowStructure> cfs;

	/**
	 * Whether or not {@link #filePath} points to a Jupyter notebook file
	 */
	private final boolean notebook;

	/**
	 * List of the indexes of cells of a Jupyter notebook in the order they are
	 * to be executed. Only valid if {@link #notebook} is {@code true}.
	 */
	private final List<Integer> cellOrder;

	/**
	 * If true, all functions and methods parsed will be considered entrypoints
	 * for the analysis. Useful for testing.
	 */
	private final boolean analyzeAll;

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
		this(filePath, notebook, Collections.emptyList(), false);
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath   file path to a Python program
	 * @param notebook   whether or not {@code filePath} points to a Jupyter
	 *                       notebook file
	 * @param analyzeAll whether or not all functions/methods should be
	 *                       considered entrypoints
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			boolean analyzeAll) {
		this(filePath, notebook, Collections.emptyList(), analyzeAll);
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
		this(filePath, notebook, List.of(cellOrder), false);
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath   file path to a Python program
	 * @param notebook   whether or not {@code filePath} points to a Jupyter
	 *                       notebook file
	 * @param cellOrder  sequence of the indexes of cells of a Jupyter notebook
	 *                       in the order they are to be executed. Only valid if
	 *                       {@code notebook} is {@code true}.
	 * @param analyzeAll whether or not all functions/methods should be
	 *                       considered entrypoints
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			boolean analyzeAll,
			Integer... cellOrder) {
		this(filePath, notebook, List.of(cellOrder), analyzeAll);
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
		this(filePath, notebook, cellOrder, false);
	}

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath   file path to a Python program
	 * @param notebook   whether or not {@code filePath} points to a Jupyter
	 *                       notebook file
	 * @param cellOrder  list of the indexes of cells of a Jupyter notebook in
	 *                       the order they are to be executed. Only valid if
	 *                       {@code notebook} is {@code true}.
	 * @param analyzeAll whether or not all functions/methods should be
	 *                       considered entrypoints
	 */
	public PyFrontend(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			boolean analyzeAll) {
		this.program = new Program(new PythonFeatures(), new PythonTypeSystem());
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
		this.analyzeAll = analyzeAll;
		this.currentUnit = new CodeUnit(new SourceCodeLocation(filePath, 0, 0),
				program, FilenameUtils.removeExtension(filePath));
		program.addUnit(currentUnit);
	}

	/**
	 * Returns the parsed file path.
	 *
	 * @return the parsed file path
	 */
	public String getFilePath() {
		return filePath;
	}

	private static String transformToCode(
			List<String> code_list) {
		StringBuilder result = new StringBuilder();
		for (String s : code_list)
			result.append(s).append("\n");
		return result.toString();
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

		LibrarySpecificationProvider.load(program);

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

		for (CFG cm : program.getAllCFGs())
			if (analyzeAll || cm.getDescriptor().getName().equals(INSTRUMENTED_MAIN_FUNCTION_NAME))
				program.addEntryPoint(cm);

		return program;
	}

	public Program toLiSAProgram() throws IOException, AnalysisSetupException {
		return toLiSAProgram(true);
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
		currentCFG = new PyCFG(buildMainCFGDescriptor(getLocation(ctx)));
		cfs = new HashSet<>();
		currentUnit.addCodeMember(currentCFG);
		Statement last_stmt = null;
		for (StmtContext stmt : IterationLogger.iterate(log, ctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Object visited;
			if (stmt.compound_stmt() != null)
				visited = visitCompound_stmt(stmt.compound_stmt());
			else
				visited = visitSimple_stmt(stmt.simple_stmt());

			if (!(visited instanceof Triple<?, ?, ?>))
				// compound statement can be a class or function definition, and
				// we don't have to add anything here
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
		cfs.forEach(currentCFG::addControlFlowStructure);
		currentCFG.simplify();
		return currentCFG;
	}

	private void addRetNodesToCurrentCFG() {
		Ret ret = new Ret(currentCFG, currentCFG.getDescriptor().getLocation());
		if (currentCFG.getNodesCount() == 0) {
			// empty method, so the ret is also the entrypoint
			currentCFG.addNode(ret, true);
		} else {
			// every non-throwing instruction that does not have a follower
			// is ending the method
			Collection<Statement> preExits = new LinkedList<>();
			for (Statement st : currentCFG.getNodes())
				if (!st.stopsExecution() && currentCFG.followersOf(st).isEmpty())
					preExits.add(st);
			if (!preExits.isEmpty()) {
				currentCFG.addNode(ret);
				for (Statement st : preExits)
					currentCFG.addEdge(new SequentialEdge(st, ret));
				for (VariableTableEntry entry : currentCFG.getDescriptor().getVariables())
					if (preExits.contains(entry.getScopeEnd()))
						entry.setScopeEnd(ret);
			}
		}
	}

	private int getLine(
			ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	private int getCol(
			ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	public SourceCodeLocation getLocation(
			ParserRuleContext ctx) {
		return new SourceCodeLocation(this.getFilePath(), getLine(ctx), getCol(ctx));
	}

	private CodeMemberDescriptor buildMainCFGDescriptor(
			SourceCodeLocation loc) {
		PyParameter[] cfgArgs = new PyParameter[] {};

		return new CodeMemberDescriptor(
				// this is just an arbitrary large value that hopefully we will
				// never encounter
				new SourceCodeLocation(loc.getSourceFile(), Integer.MAX_VALUE, Integer.MAX_VALUE),
				currentUnit,
				false,
				INSTRUMENTED_MAIN_FUNCTION_NAME,
				cfgArgs);
	}

	private CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl) {
		String funcName = funcDecl.NAME().getText();

		PyParameter[] cfgArgs = visitParameters(funcDecl.parameters());

		return new CodeMemberDescriptor(getLocation(funcDecl), currentUnit,
				currentUnit instanceof ClassUnit ? true : false,
				funcName, cfgArgs);
	}

	@Override
	public Object visitEval_input(
			Eval_inputContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public AnnotationMember visitDecorator(
			DecoratorContext ctx) {
		if (ctx.dotted_name() == null)
			throw new UnsupportedOperationException("Expecting a Dotted_nameContext in a DecoratorContext.");

		List<Expression> params = new ArrayList<>();
		String varName = ctx.dotted_name().children.get(0).getText();
		params.add(new VariableRef(this.currentCFG, getLocation(ctx), varName));

		if (ctx.arglist() != null)
			for (Expression arg : visitArglist(ctx.arglist()))
				params.add(arg);

		List<ParseTree> trees = ctx.dotted_name().children.subList(1, ctx.dotted_name().children.size());
		String target = trees.stream()
				.filter(pt -> !pt.getText().equals("."))
				.map(ParseTree::getText)
				.collect(Collectors.joining("."));
		UnresolvedCall uc = new UnresolvedCall(currentCFG, getLocation(ctx), CallType.UNKNOWN, null, target,
				params.toArray(Expression[]::new));
		return new AnnotationMember(ctx.dotted_name().getText(), new DecoratedAnnotation(params, uc));
	}

	@Override
	public Annotation visitDecorators(
			DecoratorsContext ctx) {
		List<AnnotationMember> annotationMembers = new ArrayList<>();
		for (DecoratorContext dc : ctx.decorator()) {
			AnnotationMember am = visitDecorator(dc);
			annotationMembers.add(am);
		}
		Annotation annotation = new Annotation("$decorators", annotationMembers);
		return annotation;
	}

	/*
	 * decorated : decorators (classdef | funcdef | async_funcdef) ;
	 * @param ctx the parse tree
	 * @return
	 */
	@Override
	public Object visitDecorated(
			DecoratedContext ctx) {
		if (ctx.decorators() != null) {
			NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
			Statement first = null, last = null;
			Annotation annotation = visitDecorators(ctx.decorators());
			for (AnnotationMember ann : annotation.getAnnotationMembers())
				if (ann.getValue() instanceof DecoratedAnnotation da) {
					Call c = da.getCall();
					if (first == null)
						first = c;
					block.addNode(c);
					if (last != null) {
						Edge e = new SequentialEdge(last, c);
						block.addEdge(e);
					}
					last = c;
				}

			if (ctx.classdef() != null) {
				ClassUnit classUnit = visitClassdef(ctx.classdef());
				classUnit.getAnnotations().addAnnotation(annotation);
			} else if (ctx.async_funcdef() != null) {
				PyCFG method = visitAsync_funcdef(ctx.async_funcdef());
				method.getDescriptor().getAnnotations().addAnnotation(annotation);
				FunctionDef fdef = new FunctionDef(this.currentCFG, getLocation(ctx), method);
				block.addNode(fdef);
				if (last != null) {
					Edge e = new SequentialEdge(last, fdef);
					block.addEdge(e);
				}
				last = fdef;
			} else if (ctx.funcdef() != null) {
				PyCFG method = visitFuncdef(ctx.funcdef());
				method.getDescriptor().getAnnotations().addAnnotation(annotation);
				FunctionDef fdef = new FunctionDef(this.currentCFG, getLocation(ctx), method);
				block.addNode(fdef);
				if (last != null) {
					Edge e = new SequentialEdge(last, fdef);
					block.addEdge(e);
				}
				last = fdef;
			} else {
				throw new UnsupportedStatementException("Expecting {'def', 'class', 'async'} after decorators.");
			}
			return Triple.of(first, block, last);
		}
		throw new UnsupportedStatementException("Expecting a DecoratorsContext in DecoratedContext");
	}

	@Override
	public PyCFG visitAsync_funcdef(
			Async_funcdefContext ctx) {
		log.warn("Async function definitions are not yet supported. The async def at line " + getLine(ctx) + " of file "
				+ getFilePath() + " is unsoundly translated into a def");
		return visitFuncdef(ctx.funcdef());
	}

	@Override
	public PyCFG visitFuncdef(
			FuncdefContext ctx) {
		PyCFG oldCFG = currentCFG;
		Collection<ControlFlowStructure> oldCfs = cfs;
		PyCFG newCFG = currentCFG = new PyCFG(buildCFGDescriptor(ctx));
		cfs = new HashSet<>();
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = visitSuite(ctx.suite());
		currentCFG.getNodeList().mergeWith(r.getMiddle());
		currentCFG.getEntrypoints().add(r.getLeft());
		addRetNodesToCurrentCFG();
		cfs.forEach(currentCFG::addControlFlowStructure);
		currentCFG.simplify();
		if (currentUnit instanceof ClassUnit) {
			((ClassUnit) currentUnit).addInstanceCodeMember(currentCFG);
		} else {
			currentUnit.addCodeMember(currentCFG);
		}
		currentCFG = oldCFG;
		cfs = oldCfs;
		return newCFG;
	}

	@Override
	public PyParameter[] visitParameters(
			ParametersContext ctx) {
		if (ctx.typedargslist() == null)
			return new PyParameter[0];
		return visitTypedargslist(ctx.typedargslist());
	}

	@Override
	public PyParameter[] visitTypedargslist(
			TypedargslistContext ctx) {
		List<PyParameter> pars = new LinkedList<>();
		for (TypedargContext typedArg : ctx.typedarg())
			if (pars.isEmpty())
				if (currentUnit instanceof ClassUnit) {
					pars.add(new PyParameter(getLocation(typedArg), typedArg.tfpdef().NAME().getText(),
							new ReferenceType(PyClassType.register(currentUnit.getName(), (ClassUnit) currentUnit))));
				} else
					pars.add(visitTypedarg(typedArg));
			else
				pars.add(visitTypedarg(typedArg));

		if (ctx.starargs() != null)
			pars.addAll(Arrays.asList(visitStarargs(ctx.starargs())));

		if (ctx.varkw() != null)
			pars.add(visitVarkw(ctx.varkw()));

		return pars.toArray(PyParameter[]::new);
	}

	@Override
	public PyParameter[] visitStarargs(
			Python3Parser.StarargsContext ctx) {
		List<PyParameter> pars = new LinkedList<>();
		if (ctx.varpositional() != null) {
			VarpositionalContext def = ctx.varpositional();
			pars.add(new VarPositionalParameter(getLocation(def), def.tfpdef().NAME().getText()));
		}

		/*
		 * if(ctx.varkwonly() != null) { // [,] *, ... pars.add(new
		 * StarParameter(getLocation(ctx.varkwonly()))); }
		 */

		if (ctx.typedarg() != null) {
			List<TypedargContext> def = ctx.typedarg();
			for (TypedargContext typedArg : def)
				pars.add(new KeywordOnlyParameter(visitTypedarg(typedArg)));
		}
		return pars.toArray(PyParameter[]::new);
	}

	@Override
	public PyParameter visitVarkw(
			Python3Parser.VarkwContext ctx) {
		return new VarKeywordParameter(getLocation(ctx), ctx.tfpdef().NAME().getText());
	}

	@Override
	public PyParameter visitTypedarg(
			TypedargContext ctx) {
		String typeHint = null;
		if (ctx.tfpdef().test() != null) {
			typeHint = visitTest(ctx.tfpdef().test()).toString();
		}
		if (ctx.test() == null)
			return new PyParameter(getLocation(ctx), ctx.tfpdef().NAME().getText(), Untyped.INSTANCE, null, null,
					typeHint);
		else
			return new PyParameter(getLocation(ctx), ctx.tfpdef().NAME().getText(), Untyped.INSTANCE,
					visitTest(ctx.test()), null, typeHint);
	}

	@Override
	public PyParameter visitTfpdef(
			TfpdefContext ctx) {
		return new PyParameter(getLocation(ctx), ctx.NAME().getText(), Untyped.INSTANCE);
	}

	@Override
	public Object visitVarargslist(
			VarargslistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public String visitVfpdef(
			VfpdefContext ctx) {
		return ctx.NAME().getText();
	}

	@Override
	public Object visitStmt(
			StmtContext ctx) {
		if (ctx.simple_stmt() != null)
			return visitSimple_stmt(ctx.simple_stmt());
		else
			return visitCompound_stmt(ctx.compound_stmt());
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSimple_stmt(
			Simple_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null, last = null;
		for (int i = 0; i < ctx.small_stmt().size(); i++) {
			Statement st = visitSmall_stmt(ctx.small_stmt(i));
			block.addNode(st);
			if (first == null)
				first = st;
			if (last != null)
				block.addEdge(new SequentialEdge(last, st));
			last = st;
		}

		return Triple.of(first, block, last);
	}

	@Override
	public Statement visitSmall_stmt(
			Small_stmtContext ctx) {
		if (ctx.expr_stmt() != null)
			return visitExpr_stmt(ctx.expr_stmt());
		else if (ctx.del_stmt() != null)
			return visitDel_stmt(ctx.del_stmt());
		else if (ctx.pass_stmt() != null)
			return visitPass_stmt(ctx.pass_stmt());
		else if (ctx.import_stmt() != null)
			return visitImport_stmt(ctx.import_stmt());
		else if (ctx.assert_stmt() != null)
			return visitAssert_stmt(ctx.assert_stmt());
		else if (ctx.flow_stmt() != null)
			return visitFlow_stmt(ctx.flow_stmt());
		else if (ctx.nonlocal_stmt() != null)
			return new NoOp(currentCFG, getLocation(ctx)); // TODO
		else if (ctx.global_stmt() != null)
			return new NoOp(currentCFG, getLocation(ctx)); // TODO
		throw new UnsupportedStatementException("Simple statement not yet supported");
	}

	@Override
	public Expression visitExpr_stmt(
			Expr_stmtContext ctx) {
		if (ctx.ASSIGN().size() == 0)
			if (ctx.testlist_star_expr().size() != 1)
				// augassign or annassign have been used, both not supported
				throw new UnsupportedStatementException();
			else
				return visitTestlist_star_expr(ctx.testlist_star_expr(0));

		PyAssign assign = new PyAssign(currentCFG, getLocation(ctx),
				visitTestlist_star_expr(ctx.testlist_star_expr(0)),
				visitTestlist_star_expr(ctx.testlist_star_expr(1)));
		return assign;
	}

	@Override
	public Object visitAnnassign(
			AnnassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitTestlist_star_expr(
			Testlist_star_exprContext ctx) {
		if (ctx.test().size() == 1)
			return visitTest(ctx.test(0));

		List<Expression> elements = new ArrayList<>();
		for (TestContext test : ctx.test())
			elements.add(visitTest(test));
		return new TupleCreation(currentCFG, getLocation(ctx), elements.toArray(Expression[]::new));
	}

	@Override
	public Object visitAugassign(
			AugassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitDel_stmt(
			Del_stmtContext ctx) {
		if (ctx.exprlist().star_expr().size() > 0)
			throw new UnsupportedStatementException("We support only expressions withou * in del statements");
		Statement result = new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"del",
				LeftToRightEvaluation.INSTANCE,
				visitExprlist(ctx.exprlist()).toArray(new Expression[ctx.exprlist().expr().size()]));
		return result;
	}

	@Override
	public Statement visitPass_stmt(
			Pass_stmtContext ctx) {
		return new NoOp(currentCFG, getLocation(ctx));
	}

	@Override
	public Statement visitFlow_stmt(
			Flow_stmtContext ctx) {
		if (ctx.return_stmt() != null)
			return visitReturn_stmt(ctx.return_stmt());

		if (ctx.raise_stmt() != null) {
			log.warn("Exceptions are not yet supported. The raise statement at line " + getLine(ctx) + " of file "
					+ getFilePath() + " is unsoundly translated into a return; statement");
			return new Ret(currentCFG, getLocation(ctx));
		}

		if (ctx.yield_stmt() != null) {
			List<Expression> l = extractExpressionsFromYieldArg(ctx.yield_stmt().yield_expr().yield_arg());
			return new UnresolvedCall(
					currentCFG,
					getLocation(ctx),
					CallType.STATIC,
					Program.PROGRAM_NAME,
					"yield from",
					LeftToRightEvaluation.INSTANCE,
					l.toArray(new Expression[0]));
		}

		if (ctx.continue_stmt() != null)
			return visitContinue_stmt(ctx.continue_stmt());

		if (ctx.break_stmt() != null)
			return visitBreak_stmt(ctx.break_stmt());

		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitBreak_stmt(
			Break_stmtContext ctx) {
		return new Break(currentCFG, getLocation(ctx));
	}

	@Override
	public Statement visitContinue_stmt(
			Continue_stmtContext ctx) {
		return new Continue(currentCFG, getLocation(ctx));
	}

	@Override
	public Statement visitReturn_stmt(
			Return_stmtContext ctx) {
		if (ctx.testlist() == null)
			return new Ret(currentCFG, getLocation(ctx));
		if (ctx.testlist().test().size() == 1)
			return new Return(currentCFG, getLocation(ctx), visitTest(ctx.testlist().test(0)));
		else
			return new Return(currentCFG, getLocation(ctx), new TupleCreation(
					currentCFG, getLocation(ctx),
					visitTestlist(ctx.testlist()).toArray(Expression[]::new)));
	}

	@Override
	public Object visitYield_stmt(
			Yield_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitRaise_stmt(
			Raise_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitImport_stmt(
			Import_stmtContext ctx) {
		if (ctx.import_from() != null)
			return visitImport_from(ctx.import_from());
		else
			return visitImport_name(ctx.import_name());
	}

	@Override
	public Statement visitImport_from(
			Import_fromContext ctx) {
		String name;
		if (ctx.dotted_name() != null)
			name = dottedNameToString(ctx.dotted_name());
		else
			name = ".";

		if (ctx.import_as_names() == null)
			return new FromImport(program, name, Map.of("*", "*"), currentCFG, getLocation(ctx));

		Map<String, String> components = new HashMap<>();
		for (Import_as_nameContext single : ctx.import_as_names().import_as_name()) {
			String importedComponent = single.NAME(0).getSymbol().getText();
			String as = single.NAME().size() == 2 ? single.NAME(1).getSymbol().getText() : null;
			components.put(importedComponent, as);
			imports.put(importedComponent, name + "." + importedComponent);
		}
		return new FromImport(program, name, components, currentCFG, getLocation(ctx));
	}

	@Override
	public Statement visitImport_name(
			Import_nameContext ctx) {
		Map<String, String> libs = new HashMap<>();
		for (Dotted_as_nameContext single : ctx.dotted_as_names().dotted_as_name()) {
			String importedLibrary = dottedNameToString(single.dotted_name());
			String as = single.NAME() != null ? single.NAME().getSymbol().getText() : null;
			libs.put(importedLibrary, as);
		}
		return new Import(program, libs, currentCFG, getLocation(ctx));
	}

	private String dottedNameToString(
			Dotted_nameContext dotted_name) {
		StringBuilder result = new StringBuilder();
		boolean first = true;
		for (TerminalNode name : dotted_name.NAME()) {
			if (first)
				first = false;
			else
				result.append(".");
			result.append(name.getSymbol().getText());
		}
		return result.toString();
	}

	@Override
	public Object visitImport_as_name(
			Import_as_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDotted_as_name(
			Dotted_as_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitImport_as_names(
			Import_as_namesContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDotted_as_names(
			Dotted_as_namesContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDotted_name(
			Dotted_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitGlobal_stmt(
			Global_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitNonlocal_stmt(
			Nonlocal_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitAssert_stmt(
			Assert_stmtContext ctx) {
		return new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				CallType.STATIC,
				"assert",
				Program.PROGRAM_NAME,
				LeftToRightEvaluation.INSTANCE,
				visitTestlist(ctx.testlist())
						.toArray(new Expression[ctx.testlist().test().size()]));
	}

	@Override
	public Object visitCompound_stmt(
			Compound_stmtContext ctx) {
		if (ctx.funcdef() != null)
			return this.visitFuncdef(ctx.funcdef());
		else if (ctx.if_stmt() != null)
			return this.visitIf_stmt(ctx.if_stmt());
		else if (ctx.while_stmt() != null)
			return this.visitWhile_stmt(ctx.while_stmt());
		else if (ctx.for_stmt() != null)
			return this.visitFor_stmt(ctx.for_stmt());
		else if (ctx.try_stmt() != null)
			return this.visitTry_stmt(ctx.try_stmt());
		else if (ctx.with_stmt() != null)
			return this.visitWith_stmt(ctx.with_stmt());
		else if (ctx.if_stmt() != null)
			return this.visitIf_stmt(ctx.if_stmt());
		else if (ctx.classdef() != null)
			return this.visitClassdef(ctx.classdef());
		else if (ctx.decorated() != null)
			return this.visitDecorated(ctx.decorated());
		else if (ctx.async_stmt() != null)
			return this.visitAsync_stmt(ctx.async_stmt());
		throw new UnsupportedStatementException("Statement " + ctx + " not yet supported");
	}

	@Override
	public Object visitAsync_stmt(
			Async_stmtContext ctx) {
		log.warn("Async statements are not yet supported. The async stmt at line " + getLine(ctx) + " of file "
				+ getFilePath() + " is unsoundly translated into its synchronous version.");
		if (ctx.funcdef() != null)
			return visitFuncdef(ctx.funcdef());

		if (ctx.for_stmt() != null)
			return visitFor_stmt(ctx.for_stmt());

		if (ctx.with_stmt() != null)
			return visitWith_stmt(ctx.with_stmt());

		throw new UnsupportedStatementException("Expecting with, for, def, in Async_stmtContext.");
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitIf_stmt(
			If_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement booleanGuard = visitTest(ctx.test(0));
		block.addNode(booleanGuard);

		// Created if exit node
		NoOp ifExitNode = new NoOp(currentCFG, getLocation(ctx));
		block.addNode(ifExitNode);

		// Visit if true block
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitSuite(ctx.suite(0));
		block.mergeWith(trueBlock.getMiddle());
		Statement trueEntry = trueBlock.getLeft();
		Statement trueExit = trueBlock.getRight();

		block.addEdge(new TrueEdge(booleanGuard, trueEntry));
		if (!trueExit.stopsExecution() && !(trueExit instanceof Continue) && !(trueExit instanceof Break))
			block.addEdge(new SequentialEdge(trueExit, ifExitNode));

		List<Pair<Statement, Collection<Statement>>> branches = new LinkedList<>();
		int testLenght = ctx.test().size();
		Statement lastElifGuard = booleanGuard;
		if (testLenght > 1)
			// if testLenght is >1 the context contains elif
			for (int i = 1; i < testLenght; i++) {
				Statement elifGuard = visitTest(ctx.test(i));
				block.addNode(elifGuard);
				block.addEdge(new FalseEdge(lastElifGuard, elifGuard));
				lastElifGuard = elifGuard;
				Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> elifBlock = visitSuite(ctx.suite(i));
				block.mergeWith(elifBlock.getMiddle());
				branches.add(Pair.of(elifGuard, elifBlock.getMiddle().getNodes()));
				Statement elifEntry = elifBlock.getLeft();
				Statement elifExit = elifBlock.getRight();

				block.addEdge(new TrueEdge(elifGuard, elifEntry));
				if (!elifExit.stopsExecution() && !(elifExit instanceof Continue) && !(elifExit instanceof Break))
					block.addEdge(new SequentialEdge(elifExit, ifExitNode));
			}

		// If statement with else
		Collection<Statement> falseStatements = new HashSet<>();
		if (ctx.ELSE() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitSuite(
					ctx.suite(ctx.suite().size() - 1));
			block.mergeWith(falseBlock.getMiddle());
			falseStatements.addAll(falseBlock.getMiddle().getNodes());
			Statement falseEntry = falseBlock.getLeft();
			Statement falseExit = falseBlock.getRight();

			block.addEdge(new FalseEdge(lastElifGuard, falseEntry));
			if (!falseExit.stopsExecution() && !(falseExit instanceof Continue) && !(falseExit instanceof Break))
				block.addEdge(new SequentialEdge(falseExit, ifExitNode));
		} else {
			// If statement with no else
			if (!lastElifGuard.stopsExecution() && !(lastElifGuard instanceof Continue)
					&& !(lastElifGuard instanceof Break))
				block.addEdge(new FalseEdge(lastElifGuard, ifExitNode));
		}

		for (int k = branches.size() - 1; k >= 0; k--) {
			Pair<Statement, Collection<Statement>> branch = branches.get(k);
			cfs.add(new IfThenElse(currentCFG.getNodeList(), branch.getLeft(), ifExitNode,
					branch.getRight(),
					new HashSet<>(falseStatements)));
		}
		cfs.add(new IfThenElse(currentCFG.getNodeList(), booleanGuard, ifExitNode,
				trueBlock.getMiddle().getNodes(),
				falseStatements));
		return Triple.of(booleanGuard, block, ifExitNode);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWhile_stmt(
			While_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of while
		NoOp whileExitNode = new NoOp(currentCFG, getLocation(ctx));
		block.addNode(whileExitNode);

		Statement condition = visitTest(ctx.test());
		block.addNode(condition);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitSuite(ctx.suite(0));

		// Fix Break and Continue stmt
		block.mergeWith(trueBlock.getMiddle());
		for (Statement s : trueBlock.getMiddle())
			if (s instanceof Continue) {
				for (Edge e : block.getOutgoingEdges(s))
					block.removeEdge(e);
				block.addEdge(new SequentialEdge(s, condition));
			} else if (s instanceof Break) {
				for (Edge e : block.getOutgoingEdges(s))
					block.removeEdge(e);
				block.addEdge(new SequentialEdge(s, whileExitNode));
			}
		block.addEdge(new TrueEdge(condition, trueBlock.getLeft()));
		block.addEdge(new SequentialEdge(trueBlock.getRight(), condition));

		// check if there's an else condition for the while
		Statement firstFollower;
		if (ctx.ELSE() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitSuite(ctx.suite(1));
			block.mergeWith(falseBlock.getMiddle());
			block.addEdge(new FalseEdge(condition, falseBlock.getLeft()));
			block.addEdge(new SequentialEdge(falseBlock.getRight(), whileExitNode));
			firstFollower = falseBlock.getLeft();
		} else {
			block.addEdge(new FalseEdge(condition, whileExitNode));
			firstFollower = whileExitNode;
		}

		cfs.add(new Loop(currentCFG.getNodeList(), condition, firstFollower, trueBlock.getMiddle().getNodes()));
		return Triple.of(condition, block, whileExitNode);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFor_stmt(
			For_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of for
		NoOp exit = new NoOp(currentCFG, getLocation(ctx));
		block.addNode(exit);

		List<Expression> exprs = visitExprlist(ctx.exprlist());
		Expression variable;
		if (exprs.size() == 1)
			variable = exprs.get(0);
		else
			variable = new TupleCreation(currentCFG, getLocation(ctx), exprs.toArray(Expression[]::new));

		List<Expression> list = visitTestlist(ctx.testlist());
		/*
		 * if (list.size() != 1) throw new
		 * UnsupportedStatementException("for loops with more than one test are not supported"
		 * );
		 */
		Expression collection = list.iterator().next();

		VariableRef counter = new VariableRef(
				currentCFG,
				getLocation(ctx),
				"__counter_location" + getLocation(ctx).getLine(), Int32Type.INSTANCE);
		Expression[] counter_pars = { collection, counter };

		// counter = 0;
		Assignment counter_init = new Assignment(
				currentCFG,
				getLocation(ctx),
				counter,
				new Int32Literal(currentCFG, getLocation(ctx), 0));
		block.addNode(counter_init);

		// counter < collection.size()
		UnresolvedCall condition = new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				CallType.INSTANCE,
				null,
				"__lt__",
				counter,
				new UnresolvedCall(
						currentCFG,
						getLocation(ctx),
						CallType.INSTANCE,
						null,
						"__len__",
						LeftToRightEvaluation.INSTANCE,
						collection));
		block.addNode(condition);

		// element = collection.at(counter)
		Assignment element_assignment = new Assignment(
				currentCFG,
				getLocation(ctx),
				variable,
				new UnresolvedCall(
						currentCFG,
						getLocation(ctx),
						CallType.INSTANCE,
						null,
						"__getitem__",
						LeftToRightEvaluation.INSTANCE,
						counter_pars));
		block.addNode(element_assignment);

		// counter = counter + 1;
		Assignment counter_increment = new Assignment(
				currentCFG,
				getLocation(ctx),
				counter,
				new PyAddition(
						currentCFG,
						getLocation(ctx),
						counter,
						new Int32Literal(
								currentCFG,
								getLocation(ctx),
								1)));
		block.addNode(counter_increment);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> body = visitSuite(ctx.suite(0));
		block.mergeWith(body.getMiddle());

		for (Statement s : body.getMiddle())
			if (s instanceof Continue) {
				for (Edge e : block.getOutgoingEdges(s))
					block.removeEdge(e);
				block.addEdge(new SequentialEdge(s, condition));
			} else if (s instanceof Break) {
				for (Edge e : block.getOutgoingEdges(s))
					block.removeEdge(e);
				block.addEdge(new SequentialEdge(s, exit));
			}

		block.addEdge(new SequentialEdge(counter_init, condition));
		block.addEdge(new TrueEdge(condition, element_assignment));
		block.addEdge(new SequentialEdge(element_assignment, body.getLeft()));
		block.addEdge(new SequentialEdge(body.getRight(), counter_increment));
		block.addEdge(new SequentialEdge(counter_increment, condition));
		block.addEdge(new FalseEdge(condition, exit));

		Collection<Statement> nodes = new HashSet<>(body.getMiddle().getNodes());
		nodes.add(element_assignment);
		nodes.add(counter_increment);
		cfs.add(new Loop(currentCFG.getNodeList(), condition, exit, nodes));
		return Triple.of(counter_init, block, exit);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitTry_stmt(
			Try_stmtContext ctx) {
		log.warn("Exceptions are not yet supported. The try block at line " + getLine(ctx) + " of file " + getFilePath()
				+ " is unsoundly translated considering only the code in the try block");
		return visitSuite(ctx.suite(0));
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_stmt(
			With_stmtContext ctx) {
		int withSize = ctx.with_item().size();
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> curr = visitWith_item(ctx.with_item(0));
		Statement start = curr.getLeft();
		Statement prev = curr.getRight();
		block.mergeWith(curr.getMiddle());

		for (int i = 1; i < withSize; i++) {
			curr = visitWith_item(ctx.with_item(i));
			block.mergeWith(curr.getMiddle());
			block.addEdge(new SequentialEdge(prev, curr.getLeft()));
			prev = curr.getRight();
		}

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> suite = visitSuite(ctx.suite());
		block.mergeWith(suite.getMiddle());
		block.addEdge(new SequentialEdge(prev, suite.getLeft()));

		return Triple.of(start, block, suite.getRight());
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_item(
			With_itemContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement test = visitTest(ctx.test());
		block.addNode(test);
		Statement expr = test;
		if (ctx.expr() != null) {
			expr = visitExpr(ctx.expr());
			block.addNode(expr);
			block.addEdge(new SequentialEdge(test, expr));
		}
		return Triple.of(test, block, expr);
	}

	@Override
	public Object visitExcept_clause(
			Except_clauseContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public List<Expression> visitTestlist(
			TestlistContext ctx) {
		List<Expression> result = new ArrayList<>(ctx.test().size());
		if (ctx.test().size() == 0)
			return result;
		for (TestContext e : ctx.test())
			result.add(visitTest(e));
		return result;
	}

	@Override
	public List<Expression> visitExprlist(
			ExprlistContext ctx) {
		if (!ctx.star_expr().isEmpty())
			// star expr is not supported
			throw new UnsupportedStatementException();

		List<Expression> result = new ArrayList<>(ctx.expr().size());
		if (ctx.expr().size() == 0)
			return result;
		for (ExprContext e : ctx.expr())
			result.add(visitExpr(e));
		return result;
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSuite(
			SuiteContext ctx) {
		if (ctx.simple_stmt() != null)
			return visitSimple_stmt(ctx.simple_stmt());
		else {
			NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
			Statement last = null, first = null;
			for (StmtContext element : ctx.stmt()) {
				Object parsed = visitStmt(element);
				if (!(parsed instanceof Triple<?, ?, ?>))
					// compound statement can be a class or function definition,
					// and
					// we don't have to add anything here
					continue;

				if (parsed != null) {
					@SuppressWarnings("unchecked")
					Triple<Statement, NodeList<CFG, Statement, Edge>,
							Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) parsed;
					block.mergeWith(st.getMiddle());
					if (first == null)
						// this is the first instruction
						first = st.getLeft();
					if (last != null)
						block.addEdge(new SequentialEdge(last, st.getLeft()));
					last = st.getRight();
				}
			}
			return Triple.of(first, block, last);
		}
	}

	@Override
	public Expression visitTest(
			TestContext ctx) {
		// no if into the condition
		if (ctx.IF() != null) {
			// visit the if into the condition
			Expression trueCase = visitOr_test(ctx.or_test(0));
			Expression booleanGuard = visitOr_test(ctx.or_test(1));
			Expression falseCase = visitTest(ctx.test());

			PyTernaryOperator ternary = new PyTernaryOperator(currentCFG, getLocation(ctx), booleanGuard, trueCase,
					falseCase);

			return ternary;
		} else if (ctx.lambdef() != null)
			return visitLambdef(ctx.lambdef());
		else
			return visitOr_test(ctx.or_test(0));
	}

	private List<Expression> extractNamesFromVarArgList(
			VarargslistContext varargslist) {
		List<VfpdefContext> names = varargslist.vfpdef();
		List<Expression> result = new ArrayList<>();
		if (names.size() == 0)
			return result;
		for (VfpdefContext e : names)
			result.add(new VariableRef(currentCFG, getLocation(e), visitVfpdef(e)));
		return result;
	}

	@Override
	public Expression visitTest_nocond(
			Test_nocondContext ctx) {
		if (ctx.or_test() != null)
			return visitOr_test(ctx.or_test());
		else
			return visitLambdef_nocond(ctx.lambdef_nocond());

	}

	@Override
	public Expression visitLambdef(
			LambdefContext ctx) {
		List<Expression> args;
		if (ctx.varargslist() != null)
			args = extractNamesFromVarArgList(ctx.varargslist());
		else
			args = new ArrayList<Expression>();

		Expression body = visitTest(ctx.test());
		return new LambdaExpression(
				args,
				body,
				currentCFG,
				getLocation(ctx));
	}

	@Override
	public Expression visitLambdef_nocond(
			Lambdef_nocondContext ctx) {
		List<Expression> args;
		if (ctx.varargslist() != null)
			args = extractNamesFromVarArgList(ctx.varargslist());
		else
			args = new ArrayList<Expression>();

		Expression body = visitTest_nocond(ctx.test_nocond());
		return new LambdaExpression(
				args,
				body,
				currentCFG,
				getLocation(ctx));
	}

	@Override
	public Expression visitOr_test(
			Or_testContext ctx) {
		int nAndTest = ctx.and_test().size();
		if (nAndTest == 1) {
			return visitAnd_test(ctx.and_test(0));
		} else if (nAndTest == 2) {
			return new PyOr(currentCFG, getLocation(ctx),
					visitAnd_test(ctx.and_test(0)),
					visitAnd_test(ctx.and_test(1)));
		} else {
			Expression temp = new PyOr(currentCFG, getLocation(ctx),
					visitAnd_test(ctx.and_test(nAndTest - 2)),
					visitAnd_test(ctx.and_test(nAndTest - 1)));
			nAndTest = nAndTest - 2;
			while (nAndTest > 0) {
				temp = new PyOr(currentCFG, getLocation(ctx),
						visitAnd_test(ctx.and_test(--nAndTest)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitAnd_test(
			And_testContext ctx) {
		int nNotTest = ctx.not_test().size();
		if (nNotTest == 1) {
			return visitNot_test(ctx.not_test(0));
		} else if (nNotTest == 2) {
			return new PyAnd(currentCFG, getLocation(ctx),
					visitNot_test(ctx.not_test(0)),
					visitNot_test(ctx.not_test(1)));
		} else {
			Expression temp = new PyAnd(currentCFG, getLocation(ctx),
					visitNot_test(ctx.not_test(nNotTest - 2)),
					visitNot_test(ctx.not_test(nNotTest - 1)));
			nNotTest = nNotTest - 2;
			while (nNotTest > 0) {
				temp = new PyAnd(currentCFG, getLocation(ctx),
						visitNot_test(ctx.not_test(--nNotTest)),
						temp);
			}

			return temp;
		}
	}

	@Override
	public Expression visitNot_test(
			Not_testContext ctx) {
		if (ctx.NOT() != null)
			return new Not(currentCFG, getLocation(ctx), visitNot_test(ctx.not_test()));
		else
			return visitComparison(ctx.comparison());
	}

	@Override
	public Expression visitComparison(
			ComparisonContext ctx) {
		int nExpr = ctx.expr().size();
		Expression result = null;
		switch (nExpr) {
		case 0:
			throw new UnsupportedStatementException();
		case 1:
			result = visitExpr(ctx.expr(0));
			break;
		case 2:
		default:
			Comp_opContext operator = ctx.comp_op(0);
			Expression left = visitExpr(ctx.expr(0));
			Expression right = visitExpr(ctx.expr(1));
			if (operator.EQUALS() != null)
				result = new PyEquals(currentCFG, getLocation(ctx), left, right);

			// Python greater (>)
			if (operator.GREATER_THAN() != null) {
				result = new PyGreaterThan(currentCFG, getLocation(ctx), left, right);
			}
			// Python greater equal (>=)
			if (operator.GT_EQ() != null)
				result = new PyGreaterOrEqual(currentCFG, getLocation(ctx), left, right);

			// Python in (in)
			if (operator.IN() != null)
				result = new PyIn(currentCFG, getLocation(ctx), left, right);

			// Python is (is)
			if (operator.IS() != null)
				result = new PyIs(currentCFG, getLocation(ctx), left, right);

			// Python less (<)
			if (operator.LESS_THAN() != null)
				result = new PyLessThan(currentCFG, getLocation(ctx), left, right);

			// Python less equal (<=)
			if (operator.LT_EQ() != null)
				result = new PyLessOrEqual(currentCFG, getLocation(ctx), left, right);

			// Python not (not)
			if (operator.NOT() != null)
				result = new Not(currentCFG, getLocation(ctx), left);

			// Python not equals (<>)
			if (operator.NOT_EQ_1() != null)
				result = new PyNotEqual(currentCFG, getLocation(ctx), left, right);

			// Python not equals (!=)
			if (operator.NOT_EQ_2() != null)
				result = new PyNotEqual(currentCFG, getLocation(ctx), left, right);
			break;
		}

		return result;
	}

	@Override
	public Object visitStar_expr(
			Star_exprContext ctx) {
		throw new UnsupportedStatementException();
	}

	public Expression visitExpr(
			ExprContext ctx) {
		int nXor = ctx.xor_expr().size();
		if (nXor == 1)
			// only one Xor
			return visitXor_expr(ctx.xor_expr(0));
		else if (nXor == 2)
			// two Xor
			return new PyBitwiseOr(currentCFG, getLocation(ctx),
					visitXor_expr(ctx.xor_expr(0)),
					visitXor_expr(ctx.xor_expr(1)));
		else {
			Expression temp = new PyBitwiseOr(currentCFG, getLocation(ctx),
					visitXor_expr(ctx.xor_expr(nXor - 2)),
					visitXor_expr(ctx.xor_expr(nXor - 1)));
			nXor = nXor - 2;
			// concatenate all the Xor expressions together
			while (nXor > 0) {
				temp = new PyBitwiseOr(currentCFG, getLocation(ctx),
						visitXor_expr(ctx.xor_expr(--nXor)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitXor_expr(
			Xor_exprContext ctx) {
		int nAnd = ctx.and_expr().size();
		if (nAnd == 1)
			return visitAnd_expr(ctx.and_expr(0));
		else if (nAnd == 2)
			return new PyBitwiseXor(currentCFG, getLocation(ctx),
					visitAnd_expr(ctx.and_expr(0)),
					visitAnd_expr(ctx.and_expr(1)));
		else {
			Expression temp = new PyBitwiseXor(currentCFG, getLocation(ctx),
					visitAnd_expr(ctx.and_expr(nAnd - 2)),
					visitAnd_expr(ctx.and_expr(nAnd - 1)));
			nAnd = nAnd - 2;
			// concatenate all the And expressions together
			while (nAnd > 0) {
				temp = new PyBitwiseXor(currentCFG, getLocation(ctx),
						visitAnd_expr(ctx.and_expr(--nAnd)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitAnd_expr(
			And_exprContext ctx) {
		int nShift = ctx.left_shift().size();
		if (nShift == 1)
			return visitLeft_shift(ctx.left_shift(0));
		else if (nShift == 2)
			return new PyBitwiseAnd(currentCFG, getLocation(ctx),
					visitLeft_shift(ctx.left_shift(0)),
					visitLeft_shift(ctx.left_shift(1)));
		else {
			Expression temp = new PyBitwiseAnd(currentCFG, getLocation(ctx),
					visitLeft_shift(ctx.left_shift(nShift - 2)),
					visitLeft_shift(ctx.left_shift(nShift - 1)));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyBitwiseAnd(currentCFG, getLocation(ctx),
						visitLeft_shift(ctx.left_shift(--nShift)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitLeft_shift(
			Left_shiftContext ctx) {
		int nShift = ctx.left_shift().size() + 1;
		if (nShift == 1)
			return visitRight_shift(ctx.right_shift());
		else if (nShift == 2)
			return new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
					visitRight_shift(ctx.right_shift()),
					visitLeft_shift(ctx.left_shift(0)));
		else {
			Expression temp = new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
					visitLeft_shift(ctx.left_shift(nShift - 3)),
					visitLeft_shift(ctx.left_shift(nShift - 2)));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
						visitLeft_shift(ctx.left_shift(--nShift - 1)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitRight_shift(
			Right_shiftContext ctx) {
		int nShift = ctx.right_shift().size() + 1;
		if (nShift == 1)
			return visitArith_expr(ctx.arith_expr());
		else if (nShift == 2)
			return new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
					visitArith_expr(ctx.arith_expr()),
					visitRight_shift(ctx.right_shift(0)));
		else {
			Expression temp = new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
					visitRight_shift(ctx.right_shift(nShift - 3)),
					visitRight_shift(ctx.right_shift(nShift - 2)));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
						visitRight_shift(ctx.right_shift(--nShift - 1)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitMinus(
			MinusContext ctx) {
		if (ctx.arith_expr() == null)
			return visitTerm(ctx.term());
		else
			return new Subtraction(currentCFG, getLocation(ctx),
					visitTerm(ctx.term()),
					visitArith_expr(ctx.arith_expr()));
	}

	@Override
	public Expression visitAdd(
			AddContext ctx) {
		if (ctx.arith_expr() == null)
			return visitTerm(ctx.term());
		else
			return new PyAddition(currentCFG, getLocation(ctx),
					visitTerm(ctx.term()),
					visitArith_expr(ctx.arith_expr()));
	}

	@Override
	public Expression visitArith_expr(
			Arith_exprContext ctx) {
		// check if there is minus(-) or an add(+)
		if (ctx.minus() != null)
			return visitMinus(ctx.minus());
		else if (ctx.add() != null)
			return visitAdd(ctx.add());
		else
			return visitTerm(ctx.term());
	}

	@Override
	public Expression visitMul(
			MulContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMultiplication(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitMat_mul(
			Mat_mulContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMatMul(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitDiv(
			DivContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new Division(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitMod(
			ModContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyRemainder(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitFloorDiv(
			FloorDivContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyFloorDiv(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	@Override
	public Expression visitTerm(
			TermContext ctx) {
		// check what's the operation in the context
		if (ctx.mul() != null)
			return visitMul(ctx.mul());
		else if (ctx.mat_mul() != null)
			return visitMat_mul(ctx.mat_mul());
		else if (ctx.div() != null)
			return visitDiv(ctx.div());
		else if (ctx.mod() != null)
			return visitMod(ctx.mod());
		else if (ctx.floorDiv() != null)
			return visitFloorDiv(ctx.floorDiv());
		else if (ctx.factor() != null)
			return visitFactor(ctx.factor());
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitFactor(
			FactorContext ctx) {
		if (ctx.power() != null)
			return visitPower(ctx.power());
		else if (ctx.NOT_OP() != null)
			return new PyBitwiseNot(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()));
		else if (ctx.MINUS() != null)
			return new PyMultiplication(currentCFG, getLocation(ctx),
					new Int32Literal(currentCFG, getLocation(ctx), -1),
					visitFactor(ctx.factor()));
		return visitFactor(ctx.factor());
	}

	@Override
	public Expression visitPower(
			PowerContext ctx) {
		if (ctx.POWER() != null)
			return new PyPower(currentCFG, getLocation(ctx),
					visitAtom_expr(ctx.atom_expr()),
					visitFactor(ctx.factor()));
		else
			return visitAtom_expr(ctx.atom_expr());
	}

	@Override
	public Expression visitAtom_expr(
			Atom_exprContext ctx) {
		if (ctx.AWAIT() != null)
			throw new UnsupportedStatementException("await is not supported");

		Expression accumulator = visitAtom(ctx.atom());
		List<Pair<Boolean, Expression[]>> trailers = new ArrayList<>(ctx.trailer().size());
		for (TrailerContext t : ctx.trailer())
			trailers.add(visitTrailer(t));

		if (trailers.isEmpty())
			return accumulator;

		String last_literal = accumulator instanceof VariableRef ? ((VariableRef) accumulator).getName() : null;
		// when we find a NAME in the trailer, we do not know if that is
		// supposed to be a field name or a function name
		// as the parameter list with the parentheses is in the next trailer
		// so we have to keep track of the last accumulator value before the
		// field access to swap out the field name and use it as function name
		Expression before_last_field_access = null;

		for (int i = 0; i < trailers.size(); i++) {
			Pair<Boolean, Expression[]> trailer = trailers.get(i);
			SourceCodeLocation loc = getLocation(ctx.trailer(i));

			if (trailer.getLeft() == null) {
				StringLiteral lit = (StringLiteral) trailer.getRight()[0];
				last_literal = lit.getValue();
				before_last_field_access = accumulator;
				accumulator = new PyAccessInstanceGlobal(
						currentCFG,
						loc,
						before_last_field_access,
						last_literal);
			} else if (trailer.getLeft()) {
				if (last_literal == null)
					// return new Empty(currentCFG, SyntheticLocation.INSTANCE);
					throw new UnsupportedStatementException(
							"When invoking a method we need to have always the name before the parentheses");
				List<Expression> pars = new ArrayList<>();
				if (before_last_field_access != null)
					pars.add(before_last_field_access);
				for (Expression arg : trailer.getRight())
					pars.add(arg);
				pars = convertAssignmentsToByNameParameters(pars);
				accumulator = new PyUnresolvedCall(
						currentCFG,
						loc,
						last_literal,
						pars.toArray(Expression[]::new));
				last_literal = null;
				before_last_field_access = null;
			} else {
				last_literal = null;
				if (trailer.getRight().length == 1)
					accumulator = new PySingleArrayAccess(
							currentCFG,
							loc,
							Untyped.INSTANCE,
							accumulator,
							trailer.getRight()[0]);
				else if (trailer.getRight().length == 2)
					accumulator = new PyDoubleArrayAccess(
							currentCFG,
							loc,
							Untyped.INSTANCE,
							accumulator,
							trailer.getRight()[0],
							trailer.getRight()[1]);
				else
					throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
			}
		}
		return accumulator;
	}

	private List<Expression> convertAssignmentsToByNameParameters(
			List<Expression> pars) {
		List<Expression> converted = new ArrayList<>(pars.size());
		for (Expression e : pars)
			if (!(e instanceof Assignment))
				converted.add(e);
			else
				converted.add(new NamedParameterExpression(e.getCFG(), e.getLocation(),
						((Assignment) e).getLeft().toString(), ((Assignment) e).getRight()));
		return converted;
	}

	@Override
	public Expression visitAtom(
			AtomContext ctx) {
		if (ctx.NAME() != null)
			// crete a variable
			return new VariableRef(currentCFG, getLocation(ctx), ctx.NAME().getText());
		else if (ctx.NUMBER() != null) {
			String text = ctx.NUMBER().getText().toLowerCase().replaceAll("_", "");
			if (text.endsWith("j"))
				// complex number
				throw new UnsupportedStatementException(
						"complex numbers are not supported (at " + getLocation(ctx) + ")");

			if (text.contains("e") || text.contains("."))
				// floating point
				return new Float32Literal(currentCFG, getLocation(ctx), Float.parseFloat(text));

			// integer
			if (text.startsWith("0x"))
				return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text.substring(2), 16));
			if (text.startsWith("0o"))
				return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text.substring(2), 8));
			if (text.startsWith("0b"))
				return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text.substring(2), 2));
			return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text));
		} else if (ctx.FALSE() != null)
			// create a literal false
			return new FalseLiteral(currentCFG, getLocation(ctx));
		else if (ctx.TRUE() != null)
			// create a literal true
			return new TrueLiteral(currentCFG, getLocation(ctx));
		else if (ctx.NONE() != null)
			// create a literal false
			return new PyNoneLiteral(currentCFG, getLocation(ctx));
		else if (ctx.STRING().size() > 0)
			// create a string
			return strip(getLocation(ctx), ctx.STRING(0).getText());
		else if (ctx.yield_expr() != null)
			// yield not supported
			throw new UnsupportedStatementException();
		else if (ctx.OPEN_BRACE() == null && ctx.dictorsetmaker() != null)
			return visitDictorsetmaker(ctx.dictorsetmaker());
		else if (ctx.OPEN_BRACK() != null) {
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			return new ListCreation(currentCFG, getLocation(ctx), sts.toArray(Expression[]::new));
		} else if (ctx.OPEN_PAREN() != null) {
			if (ctx.yield_expr() != null)
				throw new UnsupportedStatementException("yield expressions not supported");
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			TupleCreation tupleCreation = new TupleCreation(currentCFG, getLocation(ctx),
					sts.toArray(Expression[]::new));
			if (tupleCreation.getSubExpressions().length == 1)
				return tupleCreation.getSubExpressions()[0];
			return tupleCreation;
		} else if (ctx.OPEN_BRACE() != null) {
			// check if it is a dict or a set
			if (!isADict(ctx.dictorsetmaker())) {
				List<Expression> values = extractElementsFromSet(ctx.dictorsetmaker());
				SetCreation s = new SetCreation(currentCFG, getLocation(ctx), values.toArray(Expression[]::new));
				return s;
			}

			List<Pair<Expression, Expression>> values = extractPairsFromDict(ctx.dictorsetmaker());
			@SuppressWarnings("unchecked")
			DictionaryCreation r = new DictionaryCreation(currentCFG, getLocation(ctx),
					values.toArray(Pair[]::new));
			return r;
		} else if (ctx.ELLIPSIS() != null)
			throw new UnsupportedStatementException();
		throw new UnsupportedStatementException();
	}

	private StringLiteral strip(
			CodeLocation location,
			String string) {
		// ', ''', ", """
		if (string.startsWith("'''") && string.endsWith("'''"))
			return new PyStringLiteral(currentCFG, location, string.substring(3, string.length() - 3), "'''");
		if (string.startsWith("\"\"\"") && string.endsWith("\"\"\""))
			return new PyStringLiteral(currentCFG, location, string.substring(3, string.length() - 3), "\"\"\"");
		if (string.startsWith("'") && string.endsWith("'"))
			return new PyStringLiteral(currentCFG, location, string.substring(1, string.length() - 1), "'");
		if (string.startsWith("\"") && string.endsWith("\""))
			return new PyStringLiteral(currentCFG, location, string.substring(1, string.length() - 1), "\"");
		return new PyStringLiteral(currentCFG, location, string, "\"");
	}

	private Boolean isADict(
			DictorsetmakerContext ctx) {
		return ctx == null || ctx.test().size() == 2 * ctx.COLON().size();
	}

	private List<Pair<Expression, Expression>> extractPairsFromDict(
			DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (ctx.test().size() != 2 * ctx.COLON().size())
			throw new UnsupportedStatementException(
					"We support only initialization of dictonaries in the form of <key> : <value>");
		// it is a Dict
		for (int i = 0; i < ctx.COLON().size(); i++) {
			Expression left = visitTest(ctx.test(2 * i));
			Expression right = visitTest(ctx.test(2 * i + 1));
			result.add(Pair.of(left, right));
		}
		return result;
	}

	private List<Expression> extractElementsFromSet(
			DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < ctx.test().size(); i++) {
			Expression e = visitTest(ctx.test(i));
			result.add(e);
		}
		return result;
	}

	private List<Expression> extractExpressionsFromYieldArg(
			Yield_argContext ctx) {
		if (ctx.test() != null) {
			List<Expression> r = new ArrayList<>(1);
			r.add(visitTest(ctx.test()));
			return r;
		} else
			return visitTestlist(ctx.testlist());
	}

	private List<Expression> extractExpressionsFromTestlist_comp(
			Testlist_compContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null || ctx.testOrStar() == null || ctx.testOrStar().size() == 0)
			return result;
		for (TestOrStarContext e : ctx.testOrStar())
			result.add(visitTestOrStar(e));
		return result;
	}

	@Override
	public Expression visitTestlist_comp(
			Testlist_compContext ctx) {
		if (ctx.comp_for() != null)
			// comp_for is not supported
			throw new UnsupportedStatementException();
		return visitTestOrStar(ctx.testOrStar(0));
	}

	@Override
	public Expression visitTestOrStar(
			TestOrStarContext ctx) {
		if (ctx.star_expr() != null)
			// star expr is not supported
			throw new UnsupportedStatementException();
		return visitTest(ctx.test());
	}

	/**
	 * The Boolean distinguishes the three cases: true for arglist, false for
	 * subscripts, null for name
	 */
	@Override
	public Pair<Boolean, Expression[]> visitTrailer(
			TrailerContext ctx) {
		if (ctx.NAME() != null)
			return Pair.of(null,
					new Expression[] { new StringLiteral(currentCFG, getLocation(ctx), ctx.NAME().getText()) });
		if (ctx.arglist() != null)
			return Pair.of(true, visitArglist(ctx.arglist()));
		return Pair.of(false, visitSubscriptlist(ctx.subscriptlist()));
	}

	@Override
	public Expression[] visitSubscriptlist(
			SubscriptlistContext ctx) {
		Expression[] res = new Expression[ctx.subscript_().size()];
		for (int i = 0; i < res.length; i++)
			res[i] = visitSubscript_(ctx.subscript_(i));
		return res;
	}

	@Override
	public Expression visitSubscript_(
			Subscript_Context ctx) {
		if (ctx.COLON() != null) {
			SourceCodeLocation loc = getLocation(ctx);
			Expression left = ctx.test1() == null ? new Empty(currentCFG, loc)
					: visitTest(ctx.test1().test());
			Expression middle = ctx.test2() == null ? new Empty(currentCFG, loc)
					: visitTest(ctx.test2().test());
			Expression right = ctx.sliceop() == null || ctx.sliceop().test() == null ? new Empty(currentCFG, loc)
					: visitTest(ctx.sliceop().test());
			return new RangeValue(currentCFG, loc, left, middle, right);
		} else
			return visitTest(ctx.test());
	}

	@Override
	public Object visitSliceop(
			SliceopContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitDictorsetmaker(
			DictorsetmakerContext ctx) {
		if (ctx.COLON().size() == 0) {
			List<Expression> values = new ArrayList<>();
			for (TestContext exp : ctx.test())
				values.add(visitTest(exp));
			return new SetCreation(currentCFG, getLocation(ctx), values.toArray(Expression[]::new));
		} else
			throw new UnsupportedStatementException();
	}

	@Override
	public ClassUnit visitClassdef(
			ClassdefContext ctx) {
		Unit previous = this.currentUnit;
		String name = ctx.NAME().getSymbol().getText();
		// TODO inheritance
		ClassUnit cu = new ClassUnit(new SourceCodeLocation(name, 0, 0), program, name, true);
		cu = (ClassUnit) PyClassType.register(name, cu).getUnit(); // FIXME
																	// check
		Expression[] superclasses = ctx.arglist() != null ? visitArglist(ctx.arglist()) : new Expression[0];
		// parse anchestors
		for (Expression superclass : superclasses) {
			// if exists a class unit in the program with name
			// superclass.getText(): add it
			// to the anchestors
			String superClassName = imports.getOrDefault(superclass.toString(), superclass.toString());
			for (Unit programCu : this.program.getUnits())
				if (programCu instanceof CompilationUnit && programCu.getName().equals(superClassName))
					cu.addAncestor(((CompilationUnit) programCu));
		}
		if (cu.getImmediateAncestors().isEmpty() && LibrarySpecificationProvider.hierarchyRoot != null)
			cu.addAncestor(LibrarySpecificationProvider.hierarchyRoot);
		this.currentUnit = cu;
		parseClassBody(ctx.suite());
		program.addUnit(cu);
		this.currentUnit = previous;
		return cu;
	}

	private void parseClassBody(
			SuiteContext ctx) {
		List<Pair<VariableRef, Expression>> fields_init = new ArrayList<>();
		/*
		 * if (ctx.simple_stmt() != null) throw new
		 * UnsupportedStatementException(
		 * "Inside the body of a class we should have only field and method definitions"
		 * );
		 */
		for (StmtContext stmt : ctx.stmt()) {
			if (stmt.simple_stmt() != null) {
				Pair<VariableRef, Expression> p = parseField(stmt.simple_stmt());
				if (p.getLeft() != null)
					currentUnit.addGlobal(new Global(getLocation(ctx), currentUnit, p.getLeft().getName(), false));
				if (p.getRight() != null)
					fields_init.add(p);
			} else if (stmt.compound_stmt().funcdef() != null)
				visitFuncdef(stmt.compound_stmt().funcdef());
			else if (stmt.compound_stmt().async_stmt() != null)
				visitAsync_stmt(stmt.compound_stmt().async_stmt());
			else if (stmt.compound_stmt().decorated() != null)
				visitDecorated(stmt.compound_stmt().decorated());
		}
	}

	private Pair<VariableRef, Expression> parseField(
			Simple_stmtContext st) {
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> simple = visitSimple_stmt(st);
		Collection<Statement> nodes = simple.getMiddle().getNodes();
		if (nodes.size() != 1)
			throw new UnsupportedStatementException("Expected a single statement, got " + nodes.size());
		Statement result = nodes.iterator().next();
		if (result instanceof Assignment) {
			Assignment ass = (Assignment) result;
			Expression assigned = ass.getLeft();
			Expression expr = ass.getRight();
			if (assigned instanceof VariableRef)
				return Pair.of((VariableRef) assigned, expr);
		} else if (result instanceof VariableRef)
			return Pair.of((VariableRef) result, null);
		else if (result instanceof StringLiteral) // it is a comment
			return Pair.of(null, null);
		throw new UnsupportedStatementException(
				"Only variables or assignments of variable are supported as field declarations");
	}

	@Override
	public Expression[] visitArglist(
			ArglistContext ctx) {
		Expression[] res = new Expression[ctx.argument().size()];
		for (int i = 0; i < res.length; i++)
			res[i] = visitArgument(ctx.argument(i));
		return res;
	}

	@Override
	public Expression visitArgument(
			ArgumentContext ctx) {
		if (ctx.ASSIGN() != null)
			return new PyAssign(currentCFG, getLocation(ctx), visitTest(ctx.test(0)), visitTest(ctx.test(1)));
		else if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(ctx), visitTest(ctx.test(0)));
		else if (ctx.comp_for() != null || ctx.POWER() != null || ctx.test().size() != 1)
			return new Empty(currentCFG, getLocation(ctx));
		// throw new UnsupportedStatementException("We support only simple
		// arguments in method calls");
		// return null;
		else
			return visitTest(ctx.test(0));
	}

	@Override
	public Object visitComp_iter(
			Comp_iterContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_for(
			Comp_forContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_if(
			Comp_ifContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitEncoding_decl(
			Encoding_declContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitYield_expr(
			Yield_exprContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitYield_arg(
			Yield_argContext ctx) {
		throw new UnsupportedStatementException();
	}
}