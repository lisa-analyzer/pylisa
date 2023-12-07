package it.unive.pylisa;

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
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.Parameter;
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
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.lisa.program.cfg.statement.logic.Not;
import it.unive.lisa.program.cfg.statement.numeric.Division;
import it.unive.lisa.program.cfg.statement.numeric.Subtraction;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.VoidType;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
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
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.While_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.With_itemContext;
import it.unive.pylisa.antlr.Python3Parser.With_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Xor_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_stmtContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
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
import it.unive.pylisa.cfg.expression.PyNewObj;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PyRemainder;
import it.unive.pylisa.cfg.expression.PySingleArrayAccess;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import it.unive.pylisa.cfg.expression.PyTernaryOperator;
import it.unive.pylisa.cfg.expression.PyTypeLiteral;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.SetCreation;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyEllipsisLiteral;
import it.unive.pylisa.cfg.expression.literal.PyFloatLiteral;
import it.unive.pylisa.cfg.expression.literal.PyIntLiteral;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.statement.SimpleSuperUnresolvedCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
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

public class PyFrontend extends Python3ParserBaseVisitor<Object> {

	public static final String INSTRUMENTED_MAIN_FUNCTION_NAME = "$main";

	private static final SequentialEdge SEQUENTIAL_SINGLETON = new SequentialEdge();

	private static final Logger log = LogManager.getLogger(PyFrontend.class);

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
		this(filePath, notebook, Collections.emptyList());
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
		this(filePath, notebook, List.of(cellOrder));
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
		this.program = new Program(new PythonFeatures(), new PythonTypeSystem());
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
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

	public Program toLiSAProgram() throws IOException, AnalysisSetupException {
		log.info("PyToCFG setup...");

		PyClassType.clearAll();

		TypeSystem types = program.getTypes();
		types.registerType(PyLambdaType.INSTANCE);
		types.registerType(StringType.INSTANCE);
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
			if (cm.getDescriptor().getName().equals(INSTRUMENTED_MAIN_FUNCTION_NAME))
				program.addEntryPoint(cm);

		return program;
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
		Parameter[] cfgArgs = new Parameter[] {};

		return new CodeMemberDescriptor(loc, currentUnit, false, INSTRUMENTED_MAIN_FUNCTION_NAME, cfgArgs);
	}

	private CodeMemberDescriptor buildCFGDescriptor(
			FuncdefContext funcDecl) {
		String funcName = funcDecl.NAME().getText();

		Parameter[] cfgArgs = visitParameters(funcDecl.parameters());

		return new CodeMemberDescriptor(getLocation(funcDecl), currentUnit,
				currentUnit instanceof ClassUnit ? true : false, funcName, cfgArgs);
	}

	@Override
	public Object visitEval_input(
			Eval_inputContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDecorator(
			DecoratorContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDecorators(
			DecoratorsContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDecorated(
			DecoratedContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitAsync_funcdef(
			Async_funcdefContext ctx) {
		throw new UnsupportedStatementException();
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
	public Parameter[] visitParameters(
			ParametersContext ctx) {
		if (ctx.typedargslist() == null)
			return new Parameter[0];
		return visitTypedargslist(ctx.typedargslist());
	}

	@Override
	public Parameter[] visitTypedargslist(
			TypedargslistContext ctx) {
		if (ctx.STAR() != null || ctx.POWER() != null)
			throw new UnsupportedStatementException();
		boolean firstParam = true;
		List<Parameter> pars = new LinkedList<>();
		if (ctx.typedarg() != null)

			for (TypedargContext def : ctx.typedarg()) {
				if (firstParam) {
					if (currentUnit instanceof ClassUnit) {
						pars.add(new Parameter(getLocation(ctx), def.tfpdef().NAME().getText(),
								new ReferenceType(
										PyClassType.register(currentUnit.getName(), (ClassUnit) currentUnit))));
					} else {
						pars.add(visitTypedarg(def));
					}
					firstParam = false;
				} else {
					pars.add(visitTypedarg(def));
				}
			}

		return pars.toArray(Parameter[]::new);
	}

	@Override
	public Parameter visitTypedarg(
			TypedargContext ctx) {
		if (ctx.tfpdef().test() != null)
			throw new UnsupportedStatementException();
		if (ctx.test() == null)
			return new Parameter(getLocation(ctx), ctx.tfpdef().NAME().getText());
		else
			return new Parameter(getLocation(ctx), ctx.tfpdef().NAME().getText(), Untyped.INSTANCE,
					visitTest(ctx.test()), new Annotations());
	}

	@Override
	public Object visitTfpdef(
			TfpdefContext ctx) {
		throw new UnsupportedStatementException();
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
		else
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
					l.toArray(new Expression[0]));
		}
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitBreak_stmt(
			Break_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitContinue_stmt(
			Continue_stmtContext ctx) {
		throw new UnsupportedStatementException();
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
				visitTestlist(ctx.testlist())
						.toArray(new Expression[ctx.testlist().test().size()]));
	}

	@Override
	public Object visitCompound_stmt(
			Compound_stmtContext ctx) {
		if (ctx.funcdef() != null) {
			return this.visitFuncdef(ctx.funcdef());
		} else if (ctx.if_stmt() != null)
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
		else
			throw new UnsupportedStatementException("Statement " + ctx + " not yet supported");
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitAsync_stmt(
			Async_stmtContext ctx) {
		throw new UnsupportedStatementException();
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
		if (!trueExit.stopsExecution())
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
				if (!elifExit.stopsExecution())
					block.addEdge(new SequentialEdge(elifExit, ifExitNode));
			}

		// If statement with else
		Collection<Statement> falseStatements = new HashSet<>();
		if (ctx.ELSE() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>,
					Statement> falseBlock = visitSuite(ctx.suite(ctx.suite().size() - 1));
			block.mergeWith(falseBlock.getMiddle());
			falseStatements.addAll(falseBlock.getMiddle().getNodes());
			Statement falseEntry = falseBlock.getLeft();
			Statement falseExit = falseBlock.getRight();

			block.addEdge(new FalseEdge(lastElifGuard, falseEntry));
			if (!falseExit.stopsExecution())
				block.addEdge(new SequentialEdge(falseExit, ifExitNode));
		} else {
			// If statement with no else
			if (!lastElifGuard.stopsExecution())
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
		block.mergeWith(trueBlock.getMiddle());
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
		if (list.size() != 1)
			throw new UnsupportedStatementException("for loops with more than one test are not supported");
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
		case 1:
			result = visitExpr(ctx.expr(0));
			break;
		case 2:
			Comp_opContext operator = ctx.comp_op(0);
			Expression left = visitExpr(ctx.expr(0));
			Expression right = visitExpr(ctx.expr(1));
			String op = null;

			if (operator.EQUALS() != null)
				op = "__eq__";
			else if (operator.GREATER_THAN() != null)
				op = "__gt__";
			else if (operator.GT_EQ() != null)
				op = "__ge__";
			else if (operator.LESS_THAN() != null)
				op = "__lt__";
			else if (operator.LT_EQ() != null)
				op = "__le__";
			else if (operator.NOT_EQ_1() != null || operator.NOT_EQ_2() != null)
				op = "__ne__";
			else if (operator.IN() != null)
				result = new PyIn(currentCFG, getLocation(ctx), left, right);
			else if (operator.IS() != null)
				result = new PyIs(currentCFG, getLocation(ctx), left, right);
			else if (operator.NOT() != null)
				result = new Not(currentCFG, getLocation(ctx), left);

			if (op != null)
				result = new UnresolvedCall(currentCFG, getLocation(ctx), CallType.INSTANCE, null, op, left, right);
			break;
		default:
			throw new UnsupportedStatementException();
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
		/*
		 * atom_expr: (AWAIT)? atom trailer*; atom: ('('
		 * (yield_expr|testlist_comp)? ')' | '[' (testlist_comp)? ']' | '{'
		 * (dictorsetmaker)? '}' | NAME | NUMBER | STRING+ | '...' | 'None' |
		 * 'True' | 'False');
		 */
		if (ctx.AWAIT() != null)
			throw new UnsupportedStatementException("await is not supported");
		if (ctx.trailer().size() > 0) {
			// trailer: '(' (arglist)? ')' | '[' subscriptlist ']' | '.' NAME;
			Expression access = visitAtom(ctx.atom());
			String last_name = access instanceof VariableRef ? ((VariableRef) access).getName() : null;
			Expression previous_access = null;
			for (TrailerContext expr : ctx.trailer()) {
				if (expr.NAME() != null) {
					last_name = expr.NAME().getSymbol().getText();
					previous_access = access;
					access = new UnresolvedCall(
							currentCFG,
							getLocation(expr),
							CallType.INSTANCE,
							null,
							"__getattribute__",
							access,
							new PyStringLiteral(currentCFG, getLocation(expr), last_name, "'"));
				} else if (expr.OPEN_PAREN() != null) {
					if (last_name == null)
						throw new UnsupportedStatementException(
								"When invoking a method we need to have always the name before the parentheses");
					List<Expression> pars = new ArrayList<>();
					String method_name = last_name;
					boolean instance = access instanceof PyAccessInstanceGlobal;
					if (instance)
						pars.add(previous_access);
					if (expr.arglist() != null)
						for (ArgumentContext arg : expr.arglist().argument())
							pars.add(visitArgument(arg));

					pars = convertAssignmentsToByNameParameters(pars);
					Unit cu;
					cu = program.getUnit(method_name);
					if (cu == null) {
						cu = program.getUnit(access.toString().replace("::", "."));
						if (cu != null) {
							for (Expression par : pars) {
								if (par instanceof PyAccessInstanceGlobal) {
									pars.remove(par);
								}
							}
						}
					}
					if (cu != null && cu instanceof ClassUnit) {
						access = new PyNewObj(
								currentCFG,
								getLocation(expr),
								"__init__",
								PyClassType.register(cu.getName(), (ClassUnit) cu),
								pars.toArray(Expression[]::new));
					} else {
						access = new UnresolvedCall(
								currentCFG,
								getLocation(expr),
								instance ? CallType.UNKNOWN : CallType.STATIC,
								null,
								method_name,
								pars.toArray(Expression[]::new));
						if (method_name.equals("super") && pars.size() == 0) {
							// if super() is inside an instance method
							if (this.currentCFG.getDescriptor().isInstance()) {
								VariableTableEntry vte = currentCFG.getDescriptor().getVariables().get(0);

								Expression[] expressions = new Expression[2];
								expressions[0] = new PyTypeLiteral(this.currentCFG, getLocation(expr),
										this.currentUnit);
								expressions[1] = new VariableRef(this.currentCFG, getLocation(expr), vte.getName());
								access = new SimpleSuperUnresolvedCall(
										currentCFG,
										getLocation(expr),
										instance ? CallType.UNKNOWN : CallType.STATIC,
										null,
										method_name,
										expressions);
							}
						}
					}
					last_name = null;
					previous_access = null;
				} else if (expr.OPEN_BRACK() != null) {
					previous_access = access;
					last_name = null;
					List<Expression> indexes = extractExpressionsFromSubscriptlist(expr.subscriptlist());
					if (indexes.size() == 1)
						access = new PySingleArrayAccess(
								currentCFG,
								getLocation(expr),
								Untyped.INSTANCE,
								access,
								indexes.get(0));
					else if (indexes.size() == 2)
						access = new PyDoubleArrayAccess(
								currentCFG,
								getLocation(expr),
								Untyped.INSTANCE,
								access,
								indexes.get(0),
								indexes.get(1));
					else
						throw new UnsupportedStatementException(
								"Only array accesses with up to 2 indexes are supported");
				} else
					throw new UnsupportedStatementException();
			}
			return access;
		} else
			return visitAtom(ctx.atom());
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
			String text = ctx.NUMBER().getText().toLowerCase();
			if (text.contains("j"))
				// complex number
				throw new UnsupportedStatementException(
						"complex numbeer are not supported (at " + getLocation(ctx) + ")");

			if (text.contains("e") || text.contains("."))
				// floating point
				return new PyFloatLiteral(currentCFG, getLocation(ctx), Float.parseFloat(text));

			// integer
			return new PyIntLiteral(currentCFG, getLocation(ctx), Integer.parseInt(text));
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
			return new TupleCreation(currentCFG, getLocation(ctx), sts.toArray(Expression[]::new));
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
			return new PyEllipsisLiteral(currentCFG, getLocation(ctx));
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

	private List<Expression> extractExpressionsFromSubscriptlist(
			SubscriptlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx.subscript_().size() == 0)
			return result;
		for (Subscript_Context e : ctx.subscript_())
			result.add(visitSubscript_(e));
		return result;
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

	@Override
	public Object visitTrailer(
			TrailerContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitSubscriptlist(
			SubscriptlistContext ctx) {
		throw new UnsupportedStatementException();
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
		ArrayList<ArgumentContext> superclasses = ctx.arglist() != null ? new ArrayList<>(ctx.arglist().argument())
				: new ArrayList<>();
		// parse anchestors
		for (ArgumentContext superclass : superclasses) {
			// if exists a class unit in the program with name
			// superclass.getText(): add it to the anchestors
			for (Unit programCu : this.program.getUnits()) {
				if (programCu instanceof CompilationUnit && programCu.getName().equals(superclass.getText())) {
					cu.addAncestor(((CompilationUnit) programCu));
				}
			}
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
		if (ctx.simple_stmt() != null)
			throw new UnsupportedStatementException(
					"Inside the body of a class we should have only field and method definitions");
		for (StmtContext stmt : ctx.stmt()) {
			if (stmt.simple_stmt() != null) {
				Pair<VariableRef, Expression> p = parseField(stmt.simple_stmt());
				currentUnit.addGlobal(new Global(getLocation(ctx), currentUnit, p.getLeft().getName(), true));
				if (p.getRight() != null)
					fields_init.add(p);
			} else if (stmt.compound_stmt().funcdef() != null)
				visitFuncdef(stmt.compound_stmt().funcdef());
			else if (stmt.compound_stmt().decorated() != null) {
				log.warn("Ignoring decorator " + stmt.compound_stmt().decorated().decorators().getText()
						+ " at " + getLocation(stmt));
				DecoratedContext c = stmt.compound_stmt().decorated();
				if (c.funcdef() != null)
					visitFuncdef(c.funcdef());
				else if (c.classdef() != null)
					visitClassdef(c.classdef());
				else
					throw new UnsupportedStatementException("We support only decorated classes and methods");
			} else
				throw new UnsupportedStatementException(
						"Inside the body of a class we should have only field and method definitions");
		}
		dumpConstructor(fields_init, getLocation(ctx));
	}

	private void dumpConstructor(
			List<Pair<VariableRef, Expression>> fields_init,
			CodeLocation location) {
		if (fields_init.size() > 0) {
			PyCFG oldCFG = currentCFG;
			currentCFG = new PyCFG(new CodeMemberDescriptor(location, currentUnit, true, "<init>"));
			Statement previous = null;
			for (Pair<VariableRef, Expression> init : fields_init) {
				Statement f = new Assignment(currentCFG, init.getLeft().getLocation(), init.getLeft(), init.getRight());
				currentCFG.addNodeIfNotPresent(f, previous == null);
				if (previous != null)
					currentCFG.addEdge(new SequentialEdge(previous, f));
				previous = f;
			}
			currentUnit.addCodeMember(currentCFG);
			currentCFG = oldCFG;
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
		throw new UnsupportedStatementException(
				"Only variables or assignments of variable are supported as field declarations");
	}

	@Override
	public Object visitArglist(
			ArglistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitArgument(
			ArgumentContext ctx) {
		if (ctx.ASSIGN() != null)
			return new PyAssign(currentCFG, getLocation(ctx), visitTest(ctx.test(0)), visitTest(ctx.test(1)));
		else if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(ctx), visitTest(ctx.test(0)));
		else if (ctx.comp_for() != null || ctx.POWER() != null || ctx.test().size() != 1)
			throw new UnsupportedStatementException("We support only simple arguments in method calls");
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