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
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
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
import it.unive.pylisa.antlr.PythonLexer;
import it.unive.pylisa.antlr.PythonParser;
import it.unive.pylisa.antlr.PythonParser.Annotated_rhsContext;
import it.unive.pylisa.antlr.PythonParser.ArgumentsContext;
import it.unive.pylisa.antlr.PythonParser.Assert_stmtContext;
import it.unive.pylisa.antlr.PythonParser.AssignmentContext;
import it.unive.pylisa.antlr.PythonParser.Assignment_expressionContext;
import it.unive.pylisa.antlr.PythonParser.AtomContext;
import it.unive.pylisa.antlr.PythonParser.AugassignContext;
import it.unive.pylisa.antlr.PythonParser.Await_primaryContext;
import it.unive.pylisa.antlr.PythonParser.Bitwise_andContext;
import it.unive.pylisa.antlr.PythonParser.Bitwise_orContext;
import it.unive.pylisa.antlr.PythonParser.Bitwise_xorContext;
import it.unive.pylisa.antlr.PythonParser.BlockContext;
import it.unive.pylisa.antlr.PythonParser.Break_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Class_defContext;
import it.unive.pylisa.antlr.PythonParser.Class_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.Compare_op_bitwise_or_pairContext;
import it.unive.pylisa.antlr.PythonParser.ComparisonContext;
import it.unive.pylisa.antlr.PythonParser.Compound_stmtContext;
import it.unive.pylisa.antlr.PythonParser.ConjunctionContext;
import it.unive.pylisa.antlr.PythonParser.Continue_stmtContext;
import it.unive.pylisa.antlr.PythonParser.DecoratorsContext;
import it.unive.pylisa.antlr.PythonParser.Default_assignmentContext;
import it.unive.pylisa.antlr.PythonParser.Del_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Del_t_atomContext;
import it.unive.pylisa.antlr.PythonParser.Del_targetContext;
import it.unive.pylisa.antlr.PythonParser.DisjunctionContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_as_namesContext;
import it.unive.pylisa.antlr.PythonParser.Dotted_nameContext;
import it.unive.pylisa.antlr.PythonParser.Double_starred_kvpairContext;
import it.unive.pylisa.antlr.PythonParser.Double_starred_kvpairsContext;
import it.unive.pylisa.antlr.PythonParser.Elif_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Else_blockContext;
import it.unive.pylisa.antlr.PythonParser.ExpressionContext;
import it.unive.pylisa.antlr.PythonParser.FactorContext;
import it.unive.pylisa.antlr.PythonParser.File_inputContext;
import it.unive.pylisa.antlr.PythonParser.For_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Function_defContext;
import it.unive.pylisa.antlr.PythonParser.Function_def_rawContext;
import it.unive.pylisa.antlr.PythonParser.Global_stmtContext;
import it.unive.pylisa.antlr.PythonParser.If_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Import_fromContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_as_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_from_targetsContext;
import it.unive.pylisa.antlr.PythonParser.Import_nameContext;
import it.unive.pylisa.antlr.PythonParser.Import_stmtContext;
import it.unive.pylisa.antlr.PythonParser.InversionContext;
import it.unive.pylisa.antlr.PythonParser.Kwarg_or_starredContext;
import it.unive.pylisa.antlr.PythonParser.KwargsContext;
import it.unive.pylisa.antlr.PythonParser.LambdefContext;
import it.unive.pylisa.antlr.PythonParser.Named_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.PythonParser.ParamContext;
import it.unive.pylisa.antlr.PythonParser.Param_maybe_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Param_no_defaultContext;
import it.unive.pylisa.antlr.PythonParser.Param_with_defaultContext;
import it.unive.pylisa.antlr.PythonParser.ParametersContext;
import it.unive.pylisa.antlr.PythonParser.Pass_stmtContext;
import it.unive.pylisa.antlr.PythonParser.PowerContext;
import it.unive.pylisa.antlr.PythonParser.PrimaryContext;
import it.unive.pylisa.antlr.PythonParser.Raise_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Return_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Shift_exprContext;
import it.unive.pylisa.antlr.PythonParser.Simple_stmtContext;
import it.unive.pylisa.antlr.PythonParser.SliceContext;
import it.unive.pylisa.antlr.PythonParser.SlicesContext;
import it.unive.pylisa.antlr.PythonParser.Star_atomContext;
import it.unive.pylisa.antlr.PythonParser.Star_etcContext;
import it.unive.pylisa.antlr.PythonParser.Star_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Star_expressionsContext;
import it.unive.pylisa.antlr.PythonParser.Star_named_expressionContext;
import it.unive.pylisa.antlr.PythonParser.Star_named_expressionsContext;
import it.unive.pylisa.antlr.PythonParser.Star_targetContext;
import it.unive.pylisa.antlr.PythonParser.Star_targetsContext;
import it.unive.pylisa.antlr.PythonParser.Starred_expressionContext;
import it.unive.pylisa.antlr.PythonParser.StatementContext;
import it.unive.pylisa.antlr.PythonParser.SumContext;
import it.unive.pylisa.antlr.PythonParser.T_primaryContext;
import it.unive.pylisa.antlr.PythonParser.Target_with_star_atomContext;
import it.unive.pylisa.antlr.PythonParser.TermContext;
import it.unive.pylisa.antlr.PythonParser.Try_stmtContext;
import it.unive.pylisa.antlr.PythonParser.TupleContext;
import it.unive.pylisa.antlr.PythonParser.While_stmtContext;
import it.unive.pylisa.antlr.PythonParser.With_itemContext;
import it.unive.pylisa.antlr.PythonParser.With_stmtContext;
import it.unive.pylisa.antlr.PythonParser.Yield_exprContext;
import it.unive.pylisa.antlr.PythonParser.Yield_stmtContext;
import it.unive.pylisa.antlr.PythonParserBaseVisitor;
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
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.statement.SimpleSuperUnresolvedCall;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.NoOpFunction;
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

public class PyFrontend extends PythonParserBaseVisitor<Object> {

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

		PythonLexer lexer = null;
		try (InputStream stream = mkStream();) {
			lexer = new PythonLexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		} catch (IOException e) {
			throw new IOException("Unable to parse '" + filePath + "'", e);
		}

		PythonParser parser = new PythonParser(new CommonTokenStream(lexer));
		ParseTree tree = parser.file_input();

		visit(tree);

		PyClassType.all().forEach(types::registerType);

		for (CFG cm : program.getAllCFGs())
			if (cm.getDescriptor().getName().equals(INSTRUMENTED_MAIN_FUNCTION_NAME))
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
	public PyCFG visitFile_input(
			File_inputContext ctx) {
		currentCFG = new PyCFG(buildMainCFGDescriptor(getLocation(ctx)));
		cfs = new HashSet<>();
		currentUnit.addCodeMember(currentCFG);
		Statement last_stmt = null;
		for (StatementContext stmt : IterationLogger.iterate(log, ctx.statements().statement(), "Parsing stmt lists...",
				"Global stmt")) {
			List<Object> visitedStmts = new ArrayList<>();
			if (stmt.compound_stmt() != null)
				visitedStmts.add(visitCompound_stmt(stmt.compound_stmt()));
			else
				for (Simple_stmtContext simple : stmt.simple_stmts().simple_stmt())
					visitedStmts.add(visitSimple_stmt(simple));

			for (Object visited : visitedStmts) {
				if (!(visited instanceof Triple<?, ?, ?>))
					// compound statement can be a class or function definition,
					// and
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
		}

		addRetNodesToCurrentCFG();
		cfs.forEach(currentCFG.getDescriptor()::addControlFlowStructure);
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

		return new CodeMemberDescriptor(loc, currentUnit, false, INSTRUMENTED_MAIN_FUNCTION_NAME, cfgArgs);
	}

	private CodeMemberDescriptor buildCFGDescriptor(
			Function_def_rawContext funcDecl) {
		String funcName = funcDecl.name().getText();

		PyParameter[] cfgArgs = funcDecl.params() != null
				? visitParameters(funcDecl.params().parameters())
				: new PyParameter[0];

		return new CodeMemberDescriptor(getLocation(funcDecl), currentUnit,
				currentUnit instanceof ClassUnit ? true : false,
				funcName, cfgArgs);
	}

	public AnnotationMember visitDecorator(
			Named_expressionContext ctx) {
		Expression expr = visitNamed_expression(ctx);

		List<Expression> params = new ArrayList<>();
		UnresolvedCall uc;
		if (expr instanceof UnresolvedCall) {
			uc = (UnresolvedCall) expr;
			params.addAll(Arrays.asList(uc.getParameters()));
		} else {
			params.add(expr);
			uc = new UnresolvedCall(currentCFG, getLocation(ctx), CallType.UNKNOWN, null, "",
					params.toArray(Expression[]::new));
		}
		return new AnnotationMember(ctx.getText(), new DecoratedAnnotation(params, uc));
	}

	@Override
	public Annotation visitDecorators(
			DecoratorsContext ctx) {
		List<AnnotationMember> annotationMembers = new ArrayList<>();
		for (Named_expressionContext dc : ctx.named_expression()) {
			AnnotationMember am = visitDecorator(dc);
			annotationMembers.add(am);
		}
		Annotation annotation = new Annotation("$decorators", annotationMembers);
		return annotation;
	}

	public PyCFG visitFuncdef(
			Function_defContext ctx) {
		PyCFG method = visitFunction_def_raw(ctx.function_def_raw());
		if (ctx.decorators() != null) {
			Annotation annotation = visitDecorators(ctx.decorators());
			method.getDescriptor().getAnnotations().addAnnotation(annotation);
		}
		return method;
	}

	@Override
	public PyCFG visitFunction_def_raw(
			Function_def_rawContext ctx) {
		if (ctx.type_params() != null)
			throw new UnsupportedStatementException("generic functions are not supported");
		if (ctx.ASYNC() != null)
			log.warn("Async function definitions are not yet supported. The async def at line " + getLine(ctx)
					+ " of file " + getFilePath() + " is unsoundly translated into a def");

		PyCFG oldCFG = currentCFG;
		Collection<ControlFlowStructure> oldCfs = cfs;
		PyCFG newCFG = currentCFG = new PyCFG(buildCFGDescriptor(ctx));
		cfs = new HashSet<>();
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> r = visitBlock(ctx.block());
		currentCFG.getNodeList().mergeWith(r.getMiddle());
		currentCFG.getEntrypoints().add(r.getLeft());
		addRetNodesToCurrentCFG();
		cfs.forEach(currentCFG.getDescriptor()::addControlFlowStructure);
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
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitBlock(
			BlockContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null, last = null;

		List<Object> visitedStmts = new ArrayList<>();
		if (ctx.simple_stmts() != null) {
			for (Simple_stmtContext simple : ctx.simple_stmts().simple_stmt())
				visitedStmts.add(visitSimple_stmt(simple));
		} else {
			for (StatementContext stmt : ctx.statements().statement()) {
				if (stmt.compound_stmt() != null)
					visitedStmts.add(visitCompound_stmt(stmt.compound_stmt()));
				else
					for (Simple_stmtContext simple : stmt.simple_stmts().simple_stmt())
						visitedStmts.add(visitSimple_stmt(simple));
			}
		}

		for (Object visited : visitedStmts) {
			if (!(visited instanceof Triple<?, ?, ?>))
				// compound statement can be a class or function definition,
				// and we don't have to add anything here
				continue;

			if (visited != null) {
				@SuppressWarnings("unchecked")
				Triple<Statement, NodeList<CFG, Statement, Edge>,
						Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) visited;
				block.mergeWith(st.getMiddle());
				if (first == null)
					first = st.getLeft();
				if (last != null)
					block.addEdge(new SequentialEdge(last, st.getLeft()));
				last = st.getRight();
			}
		}
		return Triple.of(first, block, last);
	}

	@Override
	public PyParameter[] visitParameters(
			ParametersContext ctx) {
		List<PyParameter> pars = new LinkedList<>();
		if (ctx.slash_no_default() != null)
			for (Param_no_defaultContext p : ctx.slash_no_default().param_no_default())
				pars.add(buildParameter(p.param(), null, pars.isEmpty()));
		else if (ctx.slash_with_default() != null) {
			for (Param_no_defaultContext p : ctx.slash_with_default().param_no_default())
				pars.add(buildParameter(p.param(), null, pars.isEmpty()));
			for (Param_with_defaultContext p : ctx.slash_with_default().param_with_default())
				pars.add(buildParameter(p.param(), p.default_assignment(), pars.isEmpty()));
		}

		for (Param_no_defaultContext p : ctx.param_no_default())
			pars.add(buildParameter(p.param(), null, pars.isEmpty()));
		for (Param_with_defaultContext p : ctx.param_with_default())
			pars.add(buildParameter(p.param(), p.default_assignment(), pars.isEmpty()));

		if (ctx.star_etc() != null)
			pars.addAll(buildStarEtcParameters(ctx.star_etc()));

		return pars.toArray(PyParameter[]::new);
	}

	private PyParameter buildParameter(
			ParamContext param,
			Default_assignmentContext def,
			boolean first) {
		if (first && currentUnit instanceof ClassUnit)
			// the first parameter of an instance method is 'self': type it
			// with the enclosing class rather than with its annotation
			return new PyParameter(getLocation(param), param.name().getText(),
					new ReferenceType(PyClassType.register(currentUnit.getName(), (ClassUnit) currentUnit)));

		String typeHint = param.annotation() != null ? visitExpression(param.annotation().expression()).toString()
				: null;
		Expression defaultValue = def != null ? visitExpression(def.expression()) : null;
		return new PyParameter(getLocation(param), param.name().getText(), Untyped.INSTANCE, defaultValue, null,
				typeHint);
	}

	private List<PyParameter> buildStarEtcParameters(
			Star_etcContext ctx) {
		List<PyParameter> pars = new ArrayList<>();
		if (ctx.param_no_default() != null)
			pars.add(new VarPositionalParameter(getLocation(ctx.param_no_default()),
					ctx.param_no_default().param().name().getText()));
		else if (ctx.param_no_default_star_annotation() != null)
			pars.add(new VarPositionalParameter(getLocation(ctx.param_no_default_star_annotation()),
					ctx.param_no_default_star_annotation().param_star_annotation().name().getText()));

		for (Param_maybe_defaultContext p : ctx.param_maybe_default())
			pars.add(new KeywordOnlyParameter(buildParameter(p.param(), p.default_assignment(), false)));

		if (ctx.kwds() != null)
			pars.add(new VarKeywordParameter(getLocation(ctx.kwds()),
					ctx.kwds().param_no_default().param().name().getText()));

		return pars;
	}

	@Override
	public Object visitStatement(
			StatementContext ctx) {
		if (ctx.compound_stmt() != null)
			return visitCompound_stmt(ctx.compound_stmt());

		List<Object> visitedStmts = new ArrayList<>();
		for (Simple_stmtContext simple : ctx.simple_stmts().simple_stmt())
			visitedStmts.add(visitSimple_stmt(simple));

		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement first = null, last = null;
		for (Object visited : visitedStmts) {
			if (!(visited instanceof Triple<?, ?, ?>))
				continue;

			if (visited != null) {
				@SuppressWarnings("unchecked")
				Triple<Statement, NodeList<CFG, Statement, Edge>,
						Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) visited;
				block.mergeWith(st.getMiddle());
				if (first == null)
					first = st.getLeft();
				if (last != null)
					block.addEdge(new SequentialEdge(last, st.getLeft()));
				last = st.getRight();
			}
		}
		return Triple.of(first, block, last);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSimple_stmt(
			Simple_stmtContext ctx) {
		Statement result;
		if (ctx.assignment() != null)
			result = visitAssignment(ctx.assignment());
		else if (ctx.star_expressions() != null)
			result = (Statement) visitStar_expressions(ctx.star_expressions());
		else if (ctx.return_stmt() != null)
			result = visitReturn_stmt(ctx.return_stmt());
		else if (ctx.import_stmt() != null)
			result = visitImport_stmt(ctx.import_stmt());
		else if (ctx.raise_stmt() != null)
			result = (Statement) visitRaise_stmt(ctx.raise_stmt());
		else if (ctx.pass_stmt() != null)
			result = visitPass_stmt(ctx.pass_stmt());
		else if (ctx.del_stmt() != null)
			result = visitDel_stmt(ctx.del_stmt());
		else if (ctx.yield_stmt() != null)
			result = (Statement) visitYield_stmt(ctx.yield_stmt());
		else if (ctx.assert_stmt() != null)
			result = (Statement) visitAssert_stmt(ctx.assert_stmt());
		else if (ctx.break_stmt() != null)
			result = visitBreak_stmt(ctx.break_stmt());
		else if (ctx.continue_stmt() != null)
			result = visitContinue_stmt(ctx.continue_stmt());
		else if (ctx.global_stmt() != null)
			result = (Statement) visitGlobal_stmt(ctx.global_stmt());
		else if (ctx.nonlocal_stmt() != null)
			result = (Statement) visitNonlocal_stmt(ctx.nonlocal_stmt());
		else if (ctx.type_alias() != null)
			throw new UnsupportedStatementException("type alias statements are not supported");
		else
			throw new UnsupportedStatementException("Simple statement not yet supported");

		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		block.addNode(result);
		return Triple.of(result, block, result);
	}

	@Override
	public Statement visitAssignment(
			AssignmentContext ctx) {
		if (ctx.COLON() != null)
			throw new UnsupportedStatementException("annotated assignments are not supported");
		if (ctx.augassign() != null)
			throw new UnsupportedStatementException("augmented assignments are not supported");

		Expression value = visitAnnotated_rhs(ctx.annotated_rhs());
		List<Star_targetsContext> targets = ctx.star_targets();
		for (int i = targets.size() - 1; i >= 0; i--)
			value = new PyAssign(currentCFG, getLocation(ctx), visitStar_targets(targets.get(i)), value);
		return (Statement) value;
	}

	@Override
	public Expression visitAnnotated_rhs(
			Annotated_rhsContext ctx) {
		if (ctx.yield_expr() != null)
			throw new UnsupportedStatementException("yield expressions are not supported");
		return visitStar_expressions(ctx.star_expressions());
	}

	@Override
	public Expression visitStar_expressions(
			Star_expressionsContext ctx) {
		if (ctx.star_expression().size() == 1)
			return visitStar_expression(ctx.star_expression(0));
		List<Expression> elements = new ArrayList<>();
		for (Star_expressionContext e : ctx.star_expression())
			elements.add(visitStar_expression(e));
		return new TupleCreation(currentCFG, getLocation(ctx), elements.toArray(Expression[]::new));
	}

	@Override
	public Expression visitStar_expression(
			Star_expressionContext ctx) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(ctx), visitBitwise_or(ctx.bitwise_or()));
		return visitExpression(ctx.expression());
	}

	@Override
	public Expression visitStar_targets(
			Star_targetsContext ctx) {
		if (ctx.star_target().size() == 1)
			return visitTarget(ctx.star_target(0));
		List<Expression> elements = new ArrayList<>();
		for (Star_targetContext t : ctx.star_target())
			elements.add(visitTarget(t));
		return new TupleCreation(currentCFG, getLocation(ctx), elements.toArray(Expression[]::new));
	}

	private Expression visitTarget(
			Star_targetContext ctx) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(ctx), visitTarget(ctx.star_target()));
		return visitTargetWithStarAtom(ctx.target_with_star_atom());
	}

	private Expression visitTargetWithStarAtom(
			Target_with_star_atomContext ctx) {
		if (ctx.star_atom() != null)
			return visitStarAtom(ctx.star_atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return new UnresolvedCall(
					currentCFG,
					getLocation(ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(ctx), ctx.name().getText(), "'"));

		List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
		if (indexes.size() == 1)
			return new PySingleArrayAccess(currentCFG, getLocation(ctx), Untyped.INSTANCE, base, indexes.get(0));
		else if (indexes.size() == 2)
			return new PyDoubleArrayAccess(currentCFG, getLocation(ctx), Untyped.INSTANCE, base, indexes.get(0),
					indexes.get(1));
		throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
	}

	private Expression visitStarAtom(
			Star_atomContext ctx) {
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(ctx), ctx.name().getText());
		if (ctx.target_with_star_atom() != null)
			return visitTargetWithStarAtom(ctx.target_with_star_atom());
		throw new UnsupportedStatementException("Tuple/list unpacking targets are not supported");
	}

	private Expression visitTPrimary(
			T_primaryContext ctx) {
		if (ctx.t_primary() == null)
			return visitAtom(ctx.atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return new UnresolvedCall(
					currentCFG,
					getLocation(ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(ctx), ctx.name().getText(), "'"));
		else if (ctx.LSQB() != null) {
			List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
			if (indexes.size() == 1)
				return new PySingleArrayAccess(currentCFG, getLocation(ctx), Untyped.INSTANCE, base,
						indexes.get(0));
			else if (indexes.size() == 2)
				return new PyDoubleArrayAccess(currentCFG, getLocation(ctx), Untyped.INSTANCE, base,
						indexes.get(0), indexes.get(1));
			throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
		} else
			throw new UnsupportedStatementException(
					"Call/generator expressions are not supported as assignment targets");
	}

	@Override
	public Object visitAugassign(
			AugassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitDel_stmt(
			Del_stmtContext ctx) {
		List<Expression> targets = new ArrayList<>();
		for (Del_targetContext t : ctx.del_targets().del_target())
			targets.add(visitDelTarget(t));

		return new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"del",
				LeftToRightEvaluation.INSTANCE,
				targets.toArray(Expression[]::new));
	}

	private Expression visitDelTarget(
			Del_targetContext ctx) {
		if (ctx.del_t_atom() != null)
			return visitDelTAtom(ctx.del_t_atom());

		Expression base = visitTPrimary(ctx.t_primary());
		if (ctx.DOT() != null)
			return new UnresolvedCall(
					currentCFG,
					getLocation(ctx),
					CallType.INSTANCE,
					null,
					"__getattribute__",
					base,
					new PyStringLiteral(currentCFG, getLocation(ctx), ctx.name().getText(), "'"));

		List<Expression> indexes = extractExpressionsFromSlices(ctx.slices());
		if (indexes.size() == 1)
			return new PySingleArrayAccess(currentCFG, getLocation(ctx), Untyped.INSTANCE, base, indexes.get(0));
		else if (indexes.size() == 2)
			return new PyDoubleArrayAccess(currentCFG, getLocation(ctx), Untyped.INSTANCE, base, indexes.get(0),
					indexes.get(1));
		throw new UnsupportedStatementException("Only array accesses with up to 2 indexes are supported");
	}

	private Expression visitDelTAtom(
			Del_t_atomContext ctx) {
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(ctx), ctx.name().getText());
		if (ctx.del_target() != null)
			return visitDelTarget(ctx.del_target());
		throw new UnsupportedStatementException("Tuple/list del targets are not supported");
	}

	@Override
	public Statement visitPass_stmt(
			Pass_stmtContext ctx) {
		return new NoOp(currentCFG, getLocation(ctx));
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
		if (ctx.star_expressions() == null)
			return new Ret(currentCFG, getLocation(ctx));
		return new Return(currentCFG, getLocation(ctx), visitStar_expressions(ctx.star_expressions()));
	}

	@Override
	public Object visitYield_stmt(
			Yield_stmtContext ctx) {
		List<Expression> l = extractYieldArguments(ctx.yield_expr());
		return new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"yield from",
				LeftToRightEvaluation.INSTANCE,
				l.toArray(new Expression[0]));
	}

	@Override
	public Object visitRaise_stmt(
			Raise_stmtContext ctx) {
		log.warn("Exceptions are not yet supported. The raise statement at line " + getLine(ctx) + " of file "
				+ getFilePath() + " is unsoundly translated into a return; statement");
		return new Ret(currentCFG, getLocation(ctx));
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

		Import_from_targetsContext targets = ctx.import_from_targets();
		if (targets.STAR() != null)
			return new FromImport(program, name, Map.of("*", "*"), currentCFG, getLocation(ctx));

		Map<String, String> components = new HashMap<>();
		for (Import_from_as_nameContext single : targets.import_from_as_names().import_from_as_name()) {
			String importedComponent = single.name(0).getText();
			String as = single.name().size() == 2 ? single.name(1).getText() : null;
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
			String as = single.name() != null ? single.name().getText() : null;
			libs.put(importedLibrary, as);
		}
		return new Import(program, libs, currentCFG, getLocation(ctx));
	}

	private String dottedNameToString(
			Dotted_nameContext dotted_name) {
		if (dotted_name.dotted_name() == null)
			return dotted_name.name().getText();
		return dottedNameToString(dotted_name.dotted_name()) + "." + dotted_name.name().getText();
	}

	@Override
	public Object visitDotted_as_name(
			Dotted_as_nameContext ctx) {
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
		List<Expression> args = new ArrayList<>();
		for (ExpressionContext e : ctx.expression())
			args.add(visitExpression(e));
		return new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				CallType.STATIC,
				"assert",
				Program.PROGRAM_NAME,
				LeftToRightEvaluation.INSTANCE,
				args.toArray(Expression[]::new));
	}

	@Override
	public Object visitCompound_stmt(
			Compound_stmtContext ctx) {
		if (ctx.function_def() != null)
			return this.visitFuncdef(ctx.function_def());
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
		else if (ctx.class_def() != null)
			return this.visitClassdef(ctx.class_def());
		else if (ctx.match_stmt() != null)
			throw new UnsupportedStatementException("match statements are not supported");
		throw new UnsupportedStatementException("Statement " + ctx + " not yet supported");
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitIf_stmt(
			If_stmtContext ctx) {
		// flatten the (possibly nested) elif chain into a plain list of
		// (guard, block) clauses, plus a trailing else block, if any
		List<Pair<Named_expressionContext, BlockContext>> clauses = new ArrayList<>();
		clauses.add(Pair.of(ctx.named_expression(), ctx.block()));
		Elif_stmtContext elif = ctx.elif_stmt();
		Else_blockContext elseBlock = ctx.else_block();
		while (elif != null) {
			clauses.add(Pair.of(elif.named_expression(), elif.block()));
			elseBlock = elif.else_block();
			elif = elif.elif_stmt();
		}

		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement booleanGuard = visitNamed_expression(clauses.get(0).getLeft());
		block.addNode(booleanGuard);

		// Created if exit node
		NoOp ifExitNode = new NoOp(currentCFG, getLocation(ctx));
		block.addNode(ifExitNode);

		// Visit if true block
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitBlock(clauses.get(0).getRight());
		block.mergeWith(trueBlock.getMiddle());
		Statement trueEntry = trueBlock.getLeft();
		Statement trueExit = trueBlock.getRight();

		block.addEdge(new TrueEdge(booleanGuard, trueEntry));
		if (!trueExit.stopsExecution() && !(trueExit instanceof Continue) && !(trueExit instanceof Break))
			block.addEdge(new SequentialEdge(trueExit, ifExitNode));

		List<Pair<Statement, Collection<Statement>>> branches = new LinkedList<>();
		Statement lastElifGuard = booleanGuard;
		// if clauses.size() is >1 the context contains elif
		for (int i = 1; i < clauses.size(); i++) {
			Statement elifGuard = visitNamed_expression(clauses.get(i).getLeft());
			block.addNode(elifGuard);
			block.addEdge(new FalseEdge(lastElifGuard, elifGuard));
			lastElifGuard = elifGuard;
			Triple<Statement, NodeList<CFG, Statement, Edge>,
					Statement> elifBlock = visitBlock(clauses.get(i).getRight());
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
		if (elseBlock != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitBlock(elseBlock.block());
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

		Statement condition = visitNamed_expression(ctx.named_expression());
		block.addNode(condition);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitBlock(ctx.block());

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
		if (ctx.else_block() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitBlock(
					ctx.else_block().block());
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

		if (ctx.ASYNC() != null)
			log.warn("Async for loops are not yet supported. The for loop at line " + getLine(ctx) + " of file "
					+ getFilePath() + " is unsoundly translated into its synchronous version.");

		Expression variable = visitStar_targets(ctx.star_targets());
		Expression collection = visitStar_expressions(ctx.star_expressions());

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

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> body = visitBlock(ctx.block());
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
		return visitBlock(ctx.block());
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_stmt(
			With_stmtContext ctx) {
		if (ctx.ASYNC() != null)
			log.warn("Async with statements are not yet supported. The with statement at line " + getLine(ctx)
					+ " of file " + getFilePath() + " is unsoundly translated into its synchronous version.");

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

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> suite = visitBlock(ctx.block());
		block.mergeWith(suite.getMiddle());
		block.addEdge(new SequentialEdge(prev, suite.getLeft()));

		return Triple.of(start, block, suite.getRight());
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_item(
			With_itemContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		Statement test = visitExpression(ctx.expression());
		block.addNode(test);
		Statement expr = test;
		if (ctx.star_target() != null) {
			expr = visitTarget(ctx.star_target());
			block.addNode(expr);
			block.addEdge(new SequentialEdge(test, expr));
		}
		return Triple.of(test, block, expr);
	}

	@Override
	public Expression visitExpression(
			ExpressionContext ctx) {
		if (ctx.lambdef() != null)
			return visitLambdef(ctx.lambdef());
		else if (ctx.IF() != null) {
			Expression trueCase = visitDisjunction(ctx.disjunction(0));
			Expression booleanGuard = visitDisjunction(ctx.disjunction(1));
			Expression falseCase = visitExpression(ctx.expression());

			return new PyTernaryOperator(currentCFG, getLocation(ctx), booleanGuard, trueCase, falseCase);
		} else
			return visitDisjunction(ctx.disjunction(0));
	}

	@Override
	public Expression visitLambdef(
			LambdefContext ctx) {
		if (ctx.lambda_params() != null)
			throw new UnsupportedStatementException("lambda parameters are not supported");
		Expression body = visitExpression(ctx.expression());
		return new LambdaExpression(
				new ArrayList<Expression>(),
				body,
				currentCFG,
				getLocation(ctx));
	}

	@Override
	public Expression visitDisjunction(
			DisjunctionContext ctx) {
		int nConjunction = ctx.conjunction().size();
		if (nConjunction == 1) {
			return visitConjunction(ctx.conjunction(0));
		} else if (nConjunction == 2) {
			return new PyOr(currentCFG, getLocation(ctx),
					visitConjunction(ctx.conjunction(0)),
					visitConjunction(ctx.conjunction(1)));
		} else {
			Expression temp = new PyOr(currentCFG, getLocation(ctx),
					visitConjunction(ctx.conjunction(nConjunction - 2)),
					visitConjunction(ctx.conjunction(nConjunction - 1)));
			nConjunction = nConjunction - 2;
			while (nConjunction > 0) {
				temp = new PyOr(currentCFG, getLocation(ctx),
						visitConjunction(ctx.conjunction(--nConjunction)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitConjunction(
			ConjunctionContext ctx) {
		int nInversion = ctx.inversion().size();
		if (nInversion == 1) {
			return visitInversion(ctx.inversion(0));
		} else if (nInversion == 2) {
			return new PyAnd(currentCFG, getLocation(ctx),
					visitInversion(ctx.inversion(0)),
					visitInversion(ctx.inversion(1)));
		} else {
			Expression temp = new PyAnd(currentCFG, getLocation(ctx),
					visitInversion(ctx.inversion(nInversion - 2)),
					visitInversion(ctx.inversion(nInversion - 1)));
			nInversion = nInversion - 2;
			while (nInversion > 0) {
				temp = new PyAnd(currentCFG, getLocation(ctx),
						visitInversion(ctx.inversion(--nInversion)),
						temp);
			}

			return temp;
		}
	}

	@Override
	public Expression visitInversion(
			InversionContext ctx) {
		if (ctx.NOT() != null)
			return new Not(currentCFG, getLocation(ctx), visitInversion(ctx.inversion()));
		else
			return visitComparison(ctx.comparison());
	}

	@Override
	public Expression visitComparison(
			ComparisonContext ctx) {
		Expression result = visitBitwise_or(ctx.bitwise_or());
		for (Compare_op_bitwise_or_pairContext pair : ctx.compare_op_bitwise_or_pair())
			result = visitCompare_op_bitwise_or_pair(pair, result);
		return result;
	}

	private Expression visitCompare_op_bitwise_or_pair(
			Compare_op_bitwise_or_pairContext ctx,
			Expression left) {
		if (ctx.eq_bitwise_or() != null)
			return new PyEquals(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.eq_bitwise_or().bitwise_or()));
		else if (ctx.noteq_bitwise_or() != null)
			return new PyNotEqual(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.noteq_bitwise_or().bitwise_or()));
		else if (ctx.lte_bitwise_or() != null)
			return new PyLessOrEqual(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.lte_bitwise_or().bitwise_or()));
		else if (ctx.lt_bitwise_or() != null)
			return new PyLessThan(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.lt_bitwise_or().bitwise_or()));
		else if (ctx.gte_bitwise_or() != null)
			return new PyGreaterOrEqual(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.gte_bitwise_or().bitwise_or()));
		else if (ctx.gt_bitwise_or() != null)
			return new PyGreaterThan(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.gt_bitwise_or().bitwise_or()));
		else if (ctx.notin_bitwise_or() != null)
			return new Not(currentCFG, getLocation(ctx),
					new PyIn(currentCFG, getLocation(ctx), left,
							visitBitwise_or(ctx.notin_bitwise_or().bitwise_or())));
		else if (ctx.in_bitwise_or() != null)
			return new PyIn(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.in_bitwise_or().bitwise_or()));
		else if (ctx.isnot_bitwise_or() != null)
			return new Not(currentCFG, getLocation(ctx),
					new PyIs(currentCFG, getLocation(ctx), left,
							visitBitwise_or(ctx.isnot_bitwise_or().bitwise_or())));
		else if (ctx.is_bitwise_or() != null)
			return new PyIs(currentCFG, getLocation(ctx), left,
					visitBitwise_or(ctx.is_bitwise_or().bitwise_or()));
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitBitwise_or(
			Bitwise_orContext ctx) {
		if (ctx.bitwise_or() == null)
			return visitBitwise_xor(ctx.bitwise_xor());
		else
			return new PyBitwiseOr(currentCFG, getLocation(ctx),
					visitBitwise_or(ctx.bitwise_or()),
					visitBitwise_xor(ctx.bitwise_xor()));
	}

	@Override
	public Expression visitBitwise_xor(
			Bitwise_xorContext ctx) {
		if (ctx.bitwise_xor() == null)
			return visitBitwise_and(ctx.bitwise_and());
		else
			return new PyBitwiseXor(currentCFG, getLocation(ctx),
					visitBitwise_xor(ctx.bitwise_xor()),
					visitBitwise_and(ctx.bitwise_and()));
	}

	@Override
	public Expression visitBitwise_and(
			Bitwise_andContext ctx) {
		if (ctx.bitwise_and() == null)
			return visitShift_expr(ctx.shift_expr());
		else
			return new PyBitwiseAnd(currentCFG, getLocation(ctx),
					visitBitwise_and(ctx.bitwise_and()),
					visitShift_expr(ctx.shift_expr()));
	}

	public Expression visitLeft_shift(
			Shift_exprContext ctx) {
		if (ctx.shift_expr() == null)
			return visitSum(ctx.sum());
		else
			return new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
					visitShift_expr(ctx.shift_expr()),
					visitSum(ctx.sum()));
	}

	public Expression visitRight_shift(
			Shift_exprContext ctx) {
		if (ctx.shift_expr() == null)
			return visitSum(ctx.sum());
		else
			return new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
					visitShift_expr(ctx.shift_expr()),
					visitSum(ctx.sum()));
	}

	@Override
	public Expression visitShift_expr(
			Shift_exprContext ctx) {
		if (ctx.shift_expr() == null)
			return visitSum(ctx.sum());
		else if (ctx.LEFTSHIFT() != null)
			return visitLeft_shift(ctx);
		else if (ctx.RIGHTSHIFT() != null)
			return visitRight_shift(ctx);
		throw new UnsupportedStatementException();
	}

	public Expression visitMinus(
			SumContext ctx) {
		if (ctx.sum() == null)
			return visitTerm(ctx.term());
		else
			return new Subtraction(currentCFG, getLocation(ctx),
					visitSum(ctx.sum()),
					visitTerm(ctx.term()));
	}

	public Expression visitAdd(
			SumContext ctx) {
		if (ctx.sum() == null)
			return visitTerm(ctx.term());
		else
			return new PyAddition(currentCFG, getLocation(ctx),
					visitSum(ctx.sum()),
					visitTerm(ctx.term()));
	}

	@Override
	public Expression visitSum(
			SumContext ctx) {
		if (ctx.sum() == null)
			return visitTerm(ctx.term());
		else if (ctx.MINUS() != null)
			return visitMinus(ctx);
		else if (ctx.PLUS() != null)
			return visitAdd(ctx);
		throw new UnsupportedStatementException();
	}

	public Expression visitMul(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMultiplication(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitMat_mul(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMatMul(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitDiv(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new Division(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitMod(
			TermContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyRemainder(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitFloorDiv(
			TermContext ctx) {
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
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else if (ctx.STAR() != null)
			return visitMul(ctx);
		else if (ctx.AT() != null)
			return visitMat_mul(ctx);
		else if (ctx.SLASH() != null)
			return visitDiv(ctx);
		else if (ctx.PERCENT() != null)
			return visitMod(ctx);
		else if (ctx.DOUBLESLASH() != null)
			return visitFloorDiv(ctx);
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitFactor(
			FactorContext ctx) {
		if (ctx.power() != null)
			return visitPower(ctx.power());
		else if (ctx.TILDE() != null)
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
		if (ctx.DOUBLESTAR() != null)
			return new PyPower(currentCFG, getLocation(ctx),
					visitAtom_expr(ctx.await_primary()),
					visitFactor(ctx.factor()));
		else
			return visitAtom_expr(ctx.await_primary());
	}

	public Expression visitAtom_expr(
			Await_primaryContext ctx) {
		if (ctx.AWAIT() != null)
			throw new UnsupportedStatementException("await is not supported");
		return visitPrimary(ctx.primary());
	}

	@Override
	public Expression visitPrimary(
			PrimaryContext ctx) {
		List<PrimaryContext> chain = new ArrayList<>();
		PrimaryContext base = ctx;
		while (base.primary() != null) {
			chain.add(0, base);
			base = base.primary();
		}

		Expression access = visitAtom(base.atom());
		String last_name = access instanceof VariableRef ? ((VariableRef) access).getName() : null;
		Expression previous_access = null;

		for (PrimaryContext frame : chain) {
			if (frame.DOT() != null) {
				last_name = frame.name().getText();
				previous_access = access;
				access = new UnresolvedCall(
						currentCFG,
						getLocation(frame),
						CallType.INSTANCE,
						null,
						"__getattribute__",
						access,
						new PyStringLiteral(currentCFG, getLocation(frame), last_name, "'"));
			} else if (frame.LPAR() != null) {
				if (last_name == null)
					return new Empty(currentCFG, getLocation(frame));

				List<Expression> pars = extractArguments(frame.arguments());
				String method_name = last_name;
				boolean instance = access instanceof PyAccessInstanceGlobal;
				if (instance)
					pars.add(0, previous_access);

				pars = convertAssignmentsToByNameParameters(pars);
				Unit cu = program.getUnit(method_name);
				if (cu == null) {
					cu = program.getUnit(access.toString().replace("::", "."));
					if (cu == null) {
						String unitName = imports.get(access.toString());
						if (unitName != null)
							cu = program.getUnit(unitName);
					}
					if (cu != null) {
						for (Expression par : pars)
							if (par instanceof AccessInstanceGlobal)
								pars.remove(par);
					}
				}
				if (cu != null && cu instanceof ClassUnit) {
					access = new PyNewObj(
							currentCFG,
							getLocation(frame),
							"__init__",
							PyClassType.register(cu.getName(), (ClassUnit) cu),
							pars.toArray(Expression[]::new));
				} else {
					access = new UnresolvedCall(
							currentCFG,
							getLocation(frame),
							instance ? CallType.UNKNOWN : CallType.STATIC,
							null,
							method_name,
							LeftToRightEvaluation.INSTANCE,
							pars.toArray(Expression[]::new));
					if (method_name.equals("super") && pars.isEmpty()) {
						// if super() is inside an instance method
						if (this.currentCFG.getDescriptor().isInstance()) {
							VariableTableEntry vte = currentCFG.getDescriptor().getVariables().get(0);

							Expression[] expressions = new Expression[2];
							expressions[0] = new PyTypeLiteral(this.currentCFG, getLocation(frame),
									this.currentUnit);
							expressions[1] = new VariableRef(this.currentCFG, getLocation(frame), vte.getName());
							access = new SimpleSuperUnresolvedCall(
									currentCFG,
									getLocation(frame),
									instance ? CallType.UNKNOWN : CallType.STATIC,
									null,
									method_name,
									expressions);
						}
					}
				}
				last_name = null;
				previous_access = null;
			} else if (frame.LSQB() != null) {
				previous_access = access;
				last_name = null;
				List<Expression> indexes = extractExpressionsFromSlices(frame.slices());
				if (indexes.size() == 1)
					access = new PySingleArrayAccess(
							currentCFG,
							getLocation(frame),
							Untyped.INSTANCE,
							access,
							indexes.get(0));
				else if (indexes.size() == 2)
					access = new PyDoubleArrayAccess(
							currentCFG,
							getLocation(frame),
							Untyped.INSTANCE,
							access,
							indexes.get(0),
							indexes.get(1));
				else
					return NoOpFunction.build(currentCFG, getLocation(ctx), null);
			} else if (frame.genexp() != null)
				throw new UnsupportedStatementException("generator expression calls are not supported");
			else
				throw new UnsupportedStatementException();
		}
		return access;
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

	private List<Expression> extractArguments(
			ArgumentsContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null || ctx.args() == null)
			return result;
		for (int i = 0; i < ctx.args().getChildCount(); i++) {
			ParseTree child = ctx.args().getChild(i);
			if (child instanceof ExpressionContext)
				result.add(visitExpression((ExpressionContext) child));
			else if (child instanceof Assignment_expressionContext)
				result.add(visitAssignment_expression((Assignment_expressionContext) child));
			else if (child instanceof Starred_expressionContext)
				result.add(visitStarred_expression((Starred_expressionContext) child));
			else if (child instanceof KwargsContext)
				extractKwargs((KwargsContext) child, result);
		}
		return result;
	}

	private void extractKwargs(
			KwargsContext ctx,
			List<Expression> result) {
		for (Kwarg_or_starredContext k : ctx.kwarg_or_starred())
			result.add(visitKwarg_or_starred(k));
		if (!ctx.kwarg_or_double_starred().isEmpty())
			throw new UnsupportedStatementException("** kwargs unpacking is not supported");
	}

	@Override
	public Expression visitKwarg_or_starred(
			Kwarg_or_starredContext ctx) {
		if (ctx.name() != null)
			return new PyAssign(currentCFG, getLocation(ctx),
					new VariableRef(currentCFG, getLocation(ctx.name()), ctx.name().getText()),
					visitExpression(ctx.expression()));
		return visitStarred_expression(ctx.starred_expression());
	}

	@Override
	public Expression visitStarred_expression(
			Starred_expressionContext ctx) {
		return new StarExpression(currentCFG, getLocation(ctx), visitExpression(ctx.expression()));
	}

	@Override
	public Expression visitAssignment_expression(
			Assignment_expressionContext ctx) {
		return new PyAssign(currentCFG, getLocation(ctx),
				new VariableRef(currentCFG, getLocation(ctx.name()), ctx.name().getText()),
				visitExpression(ctx.expression()));
	}

	@Override
	public Expression visitNamed_expression(
			Named_expressionContext ctx) {
		if (ctx.assignment_expression() != null)
			return visitAssignment_expression(ctx.assignment_expression());
		return visitExpression(ctx.expression());
	}

	private List<Expression> extractExpressionsFromSlices(
			SlicesContext ctx) {
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < ctx.getChildCount(); i++) {
			ParseTree child = ctx.getChild(i);
			if (child instanceof SliceContext)
				result.add(visitSlice((SliceContext) child));
			else if (child instanceof Starred_expressionContext)
				result.add(visitStarred_expression((Starred_expressionContext) child));
		}
		return result;
	}

	@Override
	public Expression visitSlice(
			SliceContext ctx) {
		if (ctx.named_expression() != null)
			return visitNamed_expression(ctx.named_expression());

		SourceCodeLocation loc = getLocation(ctx);
		Expression start = new Empty(currentCFG, loc);
		Expression stop = new Empty(currentCFG, loc);
		Expression step = new Empty(currentCFG, loc);
		int colonsSeen = 0;
		for (int i = 0; i < ctx.getChildCount(); i++) {
			ParseTree child = ctx.getChild(i);
			if (child instanceof TerminalNode)
				colonsSeen++;
			else if (child instanceof ExpressionContext) {
				Expression e = visitExpression((ExpressionContext) child);
				if (colonsSeen == 0)
					start = e;
				else if (colonsSeen == 1)
					stop = e;
				else
					step = e;
			}
		}
		return new RangeValue(currentCFG, loc, start, stop, step);
	}

	private List<Expression> extractExpressionsFromStar_named_expressions(
			Star_named_expressionsContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null)
			return result;
		for (Star_named_expressionContext e : ctx.star_named_expression())
			result.add(visitStar_named_expression(e));
		return result;
	}

	@Override
	public Expression visitStar_named_expression(
			Star_named_expressionContext ctx) {
		if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(ctx), visitBitwise_or(ctx.bitwise_or()));
		return visitNamed_expression(ctx.named_expression());
	}

	private List<Pair<Expression, Expression>> extractPairsFromDict(
			Double_starred_kvpairsContext ctx) {
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (ctx == null)
			return result;
		for (Double_starred_kvpairContext e : ctx.double_starred_kvpair()) {
			if (e.kvpair() == null)
				throw new UnsupportedStatementException("** dict unpacking is not supported");
			result.add(Pair.of(visitExpression(e.kvpair().expression(0)), visitExpression(e.kvpair().expression(1))));
		}
		return result;
	}

	@Override
	public Expression visitAtom(
			AtomContext ctx) {
		if (ctx.name() != null)
			return new VariableRef(currentCFG, getLocation(ctx), ctx.name().getText());
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
			return new FalseLiteral(currentCFG, getLocation(ctx));
		else if (ctx.TRUE() != null)
			return new TrueLiteral(currentCFG, getLocation(ctx));
		else if (ctx.NONE() != null)
			return new PyNoneLiteral(currentCFG, getLocation(ctx));
		else if (ctx.strings() != null) {
			if (!ctx.strings().string().isEmpty())
				return strip(getLocation(ctx), ctx.strings().string(0).getText());
			throw new UnsupportedStatementException("formatted strings are not supported");
		} else if (ctx.tuple() != null) {
			TupleContext tuple = ctx.tuple();
			List<Expression> elements = new ArrayList<>();
			if (tuple.star_named_expression() != null)
				elements.add(visitStar_named_expression(tuple.star_named_expression()));
			elements.addAll(extractExpressionsFromStar_named_expressions(tuple.star_named_expressions()));
			TupleCreation tupleCreation = new TupleCreation(currentCFG, getLocation(ctx),
					elements.toArray(Expression[]::new));
			if (tupleCreation.getSubExpressions().length == 1)
				return tupleCreation.getSubExpressions()[0];
			return tupleCreation;
		} else if (ctx.group() != null) {
			if (ctx.group().named_expression() == null)
				throw new UnsupportedStatementException("yield expressions are not supported");
			return visitNamed_expression(ctx.group().named_expression());
		} else if (ctx.genexp() != null)
			throw new UnsupportedStatementException("generator expressions are not supported");
		else if (ctx.list() != null) {
			List<Expression> sts = extractExpressionsFromStar_named_expressions(ctx.list().star_named_expressions());
			return new ListCreation(currentCFG, getLocation(ctx), sts.toArray(Expression[]::new));
		} else if (ctx.listcomp() != null)
			throw new UnsupportedStatementException("list comprehensions are not supported");
		else if (ctx.dict() != null) {
			List<Pair<Expression, Expression>> values = extractPairsFromDict(ctx.dict().double_starred_kvpairs());
			@SuppressWarnings("unchecked")
			DictionaryCreation r = new DictionaryCreation(currentCFG, getLocation(ctx),
					values.toArray(Pair[]::new));
			return r;
		} else if (ctx.set() != null) {
			List<Expression> values = extractExpressionsFromStar_named_expressions(ctx.set().star_named_expressions());
			return new SetCreation(currentCFG, getLocation(ctx), values.toArray(Expression[]::new));
		} else if (ctx.dictcomp() != null || ctx.setcomp() != null)
			throw new UnsupportedStatementException("comprehensions are not supported");
		else if (ctx.ELLIPSIS() != null)
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

	private List<Expression> extractYieldArguments(
			Yield_exprContext ctx) {
		List<Expression> r = new ArrayList<>(1);
		if (ctx.FROM() != null)
			r.add(visitExpression(ctx.expression()));
		else if (ctx.star_expressions() != null)
			r.add(visitStar_expressions(ctx.star_expressions()));
		return r;
	}

	public ClassUnit visitClassdef(
			Class_defContext ctx) {
		ClassUnit cu = visitClass_def_raw(ctx.class_def_raw());
		if (ctx.decorators() != null) {
			Annotation annotation = visitDecorators(ctx.decorators());
			cu.getAnnotations().addAnnotation(annotation);
		}
		return cu;
	}

	@Override
	public ClassUnit visitClass_def_raw(
			Class_def_rawContext ctx) {
		if (ctx.type_params() != null)
			throw new UnsupportedStatementException("generic classes are not supported");

		Unit previous = this.currentUnit;
		String name = ctx.name().getText();
		// TODO inheritance
		ClassUnit cu = new ClassUnit(new SourceCodeLocation(name, 0, 0), program, name, true);
		List<Expression> superclasses = extractArguments(ctx.arguments());
		// parse anchestors
		for (Expression superclass : superclasses) {
			// if exists a class unit in the program with name
			// superclass's text: add it
			// to the anchestors
			String superClassName = imports.getOrDefault(superclass.toString(), superclass.toString());
			for (Unit programCu : this.program.getUnits())
				if (programCu instanceof CompilationUnit && programCu.getName().equals(superClassName))
					cu.addAncestor(((CompilationUnit) programCu));
		}
		if (cu.getImmediateAncestors().isEmpty() && LibrarySpecificationProvider.hierarchyRoot != null)
			cu.addAncestor(LibrarySpecificationProvider.hierarchyRoot);
		this.currentUnit = cu;
		parseClassBody(ctx.block());
		program.addUnit(cu);
		this.currentUnit = previous;
		return cu;
	}

	private void parseClassBody(
			BlockContext ctx) {
		List<Pair<VariableRef, Expression>> fields_init = new ArrayList<>();
		List<Simple_stmtContext> topLevel = new ArrayList<>();
		if (ctx.simple_stmts() != null)
			topLevel.addAll(ctx.simple_stmts().simple_stmt());
		if (ctx.statements() != null)
			for (StatementContext stmt : ctx.statements().statement()) {
				if (stmt.compound_stmt() == null)
					topLevel.addAll(stmt.simple_stmts().simple_stmt());
				else if (stmt.compound_stmt().function_def() != null)
					visitFuncdef(stmt.compound_stmt().function_def());
				else if (stmt.compound_stmt().class_def() != null)
					visitClassdef(stmt.compound_stmt().class_def());
			}

		for (Simple_stmtContext simple : topLevel) {
			Pair<VariableRef, Expression> p = parseField(simple);
			if (p.getLeft() != null)
				currentUnit.addGlobal(new Global(getLocation(ctx), currentUnit, p.getLeft().getName(), false));
			if (p.getRight() != null)
				fields_init.add(p);
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

}