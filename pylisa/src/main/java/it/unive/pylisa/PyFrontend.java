package it.unive.pylisa;

import static it.unive.lisa.LiSAFactory.getDefaultFor;

import java.io.ByteArrayInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.RuleContext;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.interprocedural.ContextBasedAnalysis;
import it.unive.lisa.logging.IterationLogger;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.VariableTableEntry;
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
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.call.assignment.PythonLikeAssigningStrategy;
import it.unive.lisa.program.cfg.statement.call.resolution.PythonLikeMatchingStrategy;
import it.unive.lisa.program.cfg.statement.call.resolution.RuntimeTypesMatchingStrategy;
import it.unive.lisa.program.cfg.statement.call.traversal.SingleInheritanceTraversalStrategy;
import it.unive.lisa.program.cfg.statement.comparison.GreaterThan;
import it.unive.lisa.program.cfg.statement.comparison.LessOrEqual;
import it.unive.lisa.program.cfg.statement.comparison.LessThan;
import it.unive.lisa.program.cfg.statement.comparison.NotEqual;
import it.unive.lisa.program.cfg.statement.global.AccessInstanceGlobal;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.NullLiteral;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.lisa.program.cfg.statement.logic.And;
import it.unive.lisa.program.cfg.statement.logic.Not;
import it.unive.lisa.program.cfg.statement.logic.Or;
import it.unive.lisa.program.cfg.statement.numeric.Addition;
import it.unive.lisa.program.cfg.statement.numeric.Division;
import it.unive.lisa.program.cfg.statement.numeric.Multiplication;
import it.unive.lisa.program.cfg.statement.numeric.Remainder;
import it.unive.lisa.program.cfg.statement.numeric.Subtraction;
import it.unive.lisa.program.cfg.statement.string.Equals;
import it.unive.lisa.type.NullType;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.type.common.BoolType;
import it.unive.lisa.type.common.Float32;
import it.unive.lisa.type.common.Int32;
import it.unive.lisa.type.common.StringType;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeTransformationDomain;
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
import it.unive.pylisa.cfg.PythonUnit;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Empty;
import it.unive.pylisa.cfg.expression.LambdaExpression;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyDoubleArrayAccess;
import it.unive.pylisa.cfg.expression.PyFloorDiv;
import it.unive.pylisa.cfg.expression.PyIn;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyMatMul;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PySingleArrayAccess;
import it.unive.pylisa.cfg.expression.PyXor;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.SetCreation;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.Import;
import it.unive.pylisa.cfg.type.PyLibraryType;
import it.unive.pylisa.cfg.type.PyListType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

public class PyFrontend extends Python3ParserBaseVisitor<Pair<Statement, Statement>> {

	// TODO this is not right, but its fine for now
	private static final SingleInheritanceTraversalStrategy TRAVERSAL_STRATEGY = SingleInheritanceTraversalStrategy.INSTANCE;
	private static final PythonLikeMatchingStrategy MATCHING_STRATEGY = new PythonLikeMatchingStrategy(
			RuntimeTypesMatchingStrategy.INSTANCE);
	private static final PythonLikeAssigningStrategy ASSIGN_STRATEGY = PythonLikeAssigningStrategy.INSTANCE;

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

	/**
	 * Whether or not {@link #filePath} points to a Jupyter notebook file
	 */
	private final boolean notebook;

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath file path to a Python program.
	 */
	public PyFrontend(String filePath, boolean notebook) {
		this.program = new Program();
		this.filePath = filePath;
		this.notebook = notebook;
		this.currentUnit = new PythonUnit(new SourceCodeLocation(filePath, 0, 0),
				FilenameUtils.removeExtension(filePath), true);
	}

	/**
	 * Returns the parsed file path.
	 * 
	 * @return the parsed file path
	 */
	public String getFilePath() {
		return filePath;
	}

	public static void main(String[] args) throws IOException, AnalysisException {
		String file = args[0];
		String extension = FilenameUtils.getExtension(file);
		PyFrontend translator = new PyFrontend(file, extension.equals("ipynb"));
		Program program = translator.toLiSAProgram();

		LiSAConfiguration conf = new LiSAConfiguration();
		conf.setDumpCFGs(true);
		conf.setWorkdir("workdir");
		conf.setDumpTypeInference(true);
		conf.setDumpAnalysis(true);
		conf.setInterproceduralAnalysis(new ContextBasedAnalysis<>());

		DataframeTransformationDomain domain = new DataframeTransformationDomain();
		PointBasedHeap heap = new PointBasedHeap();
		TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
		conf.setAbstractState(getDefaultFor(AbstractState.class, heap, domain, type));

		LiSA lisa = new LiSA(conf);
		lisa.run(program);
	}

	private static String transformToCode(List<String> code_list) {
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
	 * @throws IOException if {@code stream} to file cannot be read from or
	 *                         closed
	 */

	public Program toLiSAProgram() throws IOException {
		log.info("PyToCFG setup...");
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

		setupProgram(program);

		return program;
	}

	private InputStream mkStream() throws FileNotFoundException {
		if (!this.notebook)
			return new FileInputStream(getFilePath());

		Gson gson = new Gson();
		JsonReader reader = gson.newJsonReader(new FileReader(filePath));
		Map<?, ?> map = gson.fromJson(reader, Map.class);
		@SuppressWarnings("unchecked")
		ArrayList<Map<?, ?>> cells = (ArrayList<Map<?, ?>>) map.get("cells");
		StringBuilder code = new StringBuilder();
		for (Map<?, ?> l : cells) {
			String ctype = (String) l.get("cell_type");
			if (ctype.equals("code")) {
				@SuppressWarnings("unchecked")
				List<String> code_list = (List<String>) l.get("source");
				code.append(transformToCode(code_list)).append("\n");
			}
		}

		return new ByteArrayInputStream(code.toString().getBytes());
	}

	@Override
	public Pair<Statement, Statement> visit(ParseTree tree) {

		if (tree instanceof File_inputContext)
			return visitFile_input((File_inputContext) tree);

		return null;
	}

	@Override
	public Pair<Statement, Statement> visitSingle_input(Single_inputContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitFile_input(File_inputContext ctx) {
		currentCFG = new PyCFG(buildMainCFGDescriptor(getLocation(ctx)));
		program.addCFG(currentCFG);
		Statement last_stmt = null;
		for (StmtContext stmt : IterationLogger.iterate(log, ctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Compound_stmtContext comp = stmt.compound_stmt();
			Pair<Statement, Statement> visited_stmt;
			if (comp != null)
				visited_stmt = visitCompound_stmt(comp);
			else
				visited_stmt = visitSimple_stmt(stmt.simple_stmt());
			if (visited_stmt != null) {
				if (last_stmt == null) {
					Collection<Edge> outedges = new HashSet<>();
					for (Edge e : currentCFG.getEdges()) {
						if (e.getSource().equals(visited_stmt.getLeft()))
							outedges.add(e);
					}
					currentCFG.addNodeIfNotPresent(visited_stmt.getLeft(), true);
					for (Edge e : outedges)
						currentCFG.addEdge(e);
				} else
					currentCFG.addEdge(new SequentialEdge(last_stmt, visited_stmt.getLeft()));
				last_stmt = visited_stmt.getRight();
			}
		}

		addRetNodesToCurrentCFG();
		return null;
	}

	private void addRetNodesToCurrentCFG() {
		if (currentCFG.getAllExitpoints().isEmpty()) {
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
				currentCFG.addNode(ret);
				for (Statement st : preExits)
					currentCFG.addEdge(new SequentialEdge(st, ret));

				for (VariableTableEntry entry : currentCFG.getDescriptor().getVariables())
					if (preExits.contains(entry.getScopeEnd()))
						entry.setScopeEnd(ret);
			}
		}

		currentCFG.simplify();
	}

	private int getLine(ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	private int getCol(ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	public SourceCodeLocation getLocation(ParserRuleContext ctx) {
		return new SourceCodeLocation(this.getFilePath(), getLine(ctx), getCol(ctx));
	}

	private CFGDescriptor buildMainCFGDescriptor(SourceCodeLocation loc) {
		String funcName = "main";
		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(loc, currentUnit, false, funcName, cfgArgs);
	}

	private CFGDescriptor buildCFGDescriptor(FuncdefContext funcDecl) {
		String funcName = funcDecl.NAME().getText();

		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(getLocation(funcDecl), currentUnit, false, funcName, cfgArgs);
	}

	@Override
	public Pair<Statement, Statement> visitEval_input(Eval_inputContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDecorator(DecoratorContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDecorators(DecoratorsContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDecorated(DecoratedContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitAsync_funcdef(Async_funcdefContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitFuncdef(FuncdefContext ctx) {
		PyCFG oldCFG = currentCFG;
		currentCFG = new PyCFG(buildCFGDescriptor(ctx));
		program.addCFG(currentCFG);
		Pair<Statement, Statement> r = visitSuite(ctx.suite());
		currentCFG.addNodeIfNotPresent(r.getLeft(), true);
		addRetNodesToCurrentCFG();
		PyCFG result = currentCFG;
		currentCFG = oldCFG;
		currentUnit.addCFG(result);
		return null;
	}

	@Override
	public Pair<Statement, Statement> visitParameters(ParametersContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitTypedargslist(TypedargslistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitTfpdef(TfpdefContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitVarargslist(VarargslistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitVfpdef(VfpdefContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitStmt(StmtContext ctx) {
		if (ctx.simple_stmt() != null)
			return visitSimple_stmt(ctx.simple_stmt());
		else
			return visitCompound_stmt(ctx.compound_stmt());
	}

	@Override
	public Pair<Statement, Statement> visitSimple_stmt(Simple_stmtContext ctx) {
		return visitListOfSmallStatements(ctx.small_stmt());
	}

	@Override
	public Pair<Statement, Statement> visitSmall_stmt(Small_stmtContext ctx) {
		if (ctx.expr_stmt() != null)
			return addToCFGAndReturn(visitExpr_stmt(ctx.expr_stmt()));
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

	private Pair<Statement, Statement> addToCFGAndReturn(Pair<Statement, Statement> pair) {
		currentCFG.addNodeIfNotPresent(pair.getLeft());
		if (pair.getLeft() != pair.getRight())
			currentCFG.addNodeIfNotPresent(pair.getRight());
		return pair;
	}

	@Override
	public Pair<Statement, Statement> visitExpr_stmt(Expr_stmtContext ctx) {

		Statement assegnazione;

		// � un assegnazione
		if (ctx.ASSIGN().size() > 0) {
			assegnazione = createAssign(visitTestlist_star_expr(ctx.testlist_star_expr(0)),
					visitTestlist_star_expr(ctx.testlist_star_expr(1)), getLocation(ctx));
			currentCFG.addNodeIfNotPresent(assegnazione);
		} else
			return visitTestlistStarExpr(ctx);
		return createPairFromSingle(assegnazione);
	}

	@Override
	public Pair<Statement, Statement> visitAnnassign(AnnassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitTestlist_star_expr(Testlist_star_exprContext ctx) {
		if (ctx.test().size() == 1) {
			return createPairFromSingle(checkAndExtractSingleStatement(visitTest(ctx.test(0))));
		}
		List<Expression> elements = new ArrayList<>();
		for (TestContext test : ctx.test())
			elements.add(checkAndExtractSingleExpression(visitTest(test)));
		return createPairFromSingle(
				new TupleCreation(currentCFG, getLocation(ctx), elements.toArray(Expression[]::new)));
	}

	@Override
	public Pair<Statement, Statement> visitAugassign(AugassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDel_stmt(Del_stmtContext ctx) {
		if (ctx.exprlist().star_expr().size() > 0)
			throw new UnsupportedStatementException("We support only expressions withou * in del statements");
		Statement result = new UnresolvedCall(
				currentCFG,
				getLocation(ctx),
				ASSIGN_STRATEGY,
				MATCHING_STRATEGY,
				TRAVERSAL_STRATEGY,
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"del",
				extractExpressionsFromExprlist(ctx.exprlist()).toArray(new Expression[ctx.exprlist().expr().size()]));
		currentCFG.addNodeIfNotPresent(result);
		return createPairFromSingle(result);
	}

	@Override
	public Pair<Statement, Statement> visitPass_stmt(Pass_stmtContext ctx) {
		Statement st = new NoOp(currentCFG, getLocation(ctx));
		currentCFG.addNodeIfNotPresent(st);
		return createPairFromSingle(st);
	}

	@Override
	public Pair<Statement, Statement> visitFlow_stmt(Flow_stmtContext ctx) {
		if (ctx.return_stmt() != null)
			return visitReturn_stmt(ctx.return_stmt());
		if (ctx.raise_stmt() != null) {
			log.warn("Exceptions are not yet supported. The raise statement at line " + getLine(ctx) + " of file "
					+ getFilePath() + " is unsoundly translated into a return; statement");
			return addToCFGAndReturn(createPairFromSingle(new Ret(currentCFG, getLocation(ctx))));
		}
		if (ctx.yield_stmt() != null) {
			List<Expression> l = extractExpressionsFromYieldArg(ctx.yield_stmt().yield_expr().yield_arg());
			return addToCFGAndReturn(createPairFromSingle(
					new UnresolvedCall(
							currentCFG,
							getLocation(ctx),
							ASSIGN_STRATEGY,
							MATCHING_STRATEGY,
							TRAVERSAL_STRATEGY,
							CallType.STATIC,
							Program.PROGRAM_NAME,
							"yield from",
							l.toArray(new Expression[0]))));
		}
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitBreak_stmt(Break_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitContinue_stmt(Continue_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitReturn_stmt(Return_stmtContext ctx) {
		if (ctx.testlist() == null)
			return addToCFGAndReturn(createPairFromSingle(new Ret(currentCFG, getLocation(ctx))));
		else {
			if (ctx.testlist().test().size() == 1)
				return addToCFGAndReturn(createPairFromSingle(new Return(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitTestlist(ctx.testlist())))));
			else
				return addToCFGAndReturn(createPairFromSingle(new TupleCreation(
						currentCFG, getLocation(ctx),
						extractExpressionsFromTestlist(ctx.testlist()).toArray(Expression[]::new))));
		}
	}

	@Override
	public Pair<Statement, Statement> visitYield_stmt(Yield_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitRaise_stmt(Raise_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitImport_stmt(Import_stmtContext ctx) {
		if (ctx.import_from() != null) {
			return visitImport_from(ctx.import_from());
		} else {
			return visitImport_name(ctx.import_name());
		}
	}

	/**
	 * This method transforms a single rule to a sequence of statements
	 * representing it
	 *
	 * @param ctx              the context that internally contains a list of
	 *                             something, and that should be translated into
	 *                             a sequence of statements
	 * @param defaultStatement the default statement to be returned if the list
	 *                             is empty
	 * @param condition        if false, the method returns the default
	 *                             statement, otherwise it processes the list
	 * @param getList          returns the list of the elements to be processed
	 * @param extractStatement transforms an element into a statement
	 *                             representing it
	 * @param <T1>             the context to be processed
	 * @param <T2>             the elements to be transformed into statements
	 * 
	 * @return the first and last statement produced when transforming the
	 *             context into statements
	 */
	private <T1 extends RuleContext, T2> Pair<Statement, Statement> visitListOfContexts(T1 ctx,
			Statement defaultStatement, Function<T1, Boolean> condition, Function<T1, List<T2>> getList,
			Function<T2, Statement> extractStatement) {
		if (condition.apply(ctx)) {
			Statement first = null;
			Statement prev;
			Statement last = null;
			for (T2 single : getList.apply(ctx)) {
				prev = last;
				last = extractStatement.apply(single);
				currentCFG.addNodeIfNotPresent(last, false);
				if (prev != null)
					currentCFG.addEdge(new SequentialEdge(prev, last));
				if (first == null)
					first = last;
			}
			return Pair.of(first, last);
		} else
			return createPairFromSingle(defaultStatement);
	}

	@Override
	public Pair<Statement, Statement> visitImport_from(Import_fromContext ctx) {
		String name;
		if (ctx.dotted_name() != null)
			name = dottedNameToString(ctx.dotted_name());
		else
			name = ".";

		Statement def = new FromImport(name, "*", "*", currentCFG, getLocation(ctx));

		return visitListOfContexts(ctx, def,
				c -> c.import_as_names() != null,
				c -> c.import_as_names().import_as_name(),
				s -> {
					String component = s.NAME(0).getSymbol().getText();
					String asName = s.NAME(1) != null ? s.NAME(1).getSymbol().getText() : component;
					return new FromImport(name, component, asName, currentCFG, getLocation(s));
				});
	}

	@Override
	public Pair<Statement, Statement> visitImport_name(Import_nameContext ctx) {
		return visitListOfContexts(
				ctx,
				null,
				c -> true,
				c -> c.dotted_as_names().dotted_as_name(),
				c -> {
					String importedLibrary = dottedNameToString(c.dotted_name());
					String as = c.NAME() != null ? c.NAME().getSymbol().getText() : importedLibrary;
					return new Import(importedLibrary, as, currentCFG, getLocation(c));
				});
	}

	private String dottedNameToString(Dotted_nameContext dotted_name) {
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
	public Pair<Statement, Statement> visitImport_as_name(Import_as_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDotted_as_name(Dotted_as_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitImport_as_names(Import_as_namesContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDotted_as_names(Dotted_as_namesContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDotted_name(Dotted_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitGlobal_stmt(Global_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitNonlocal_stmt(Nonlocal_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitAssert_stmt(Assert_stmtContext ctx) {
		return addToCFGAndReturn(
				createPairFromSingle(
						new UnresolvedCall(
								currentCFG,
								getLocation(ctx),
								ASSIGN_STRATEGY,
								MATCHING_STRATEGY,
								TRAVERSAL_STRATEGY,
								CallType.STATIC,
								"assert",
								Program.PROGRAM_NAME,
								extractExpressionsFromListOfTests(ctx.test())
										.toArray(new Expression[ctx.test().size()]))));
	}

	@Override
	public Pair<Statement, Statement> visitCompound_stmt(Compound_stmtContext ctx) {
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
	public Pair<Statement, Statement> visitAsync_stmt(Async_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	private Statement checkAndExtractSingleStatement(Pair<Statement, Statement> parsed) {
		if (parsed.getLeft() != parsed.getRight())
			throw new UnsupportedStatementException("It is not supported to have multiple expressions here");
		return parsed.getLeft();
	}

	private Expression checkAndExtractSingleExpression(Pair<Statement, Statement> parsed) {
		Statement result = checkAndExtractSingleStatement(parsed);
		if (!(result instanceof Expression))
			throw new UnsupportedStatementException("An expression is expected here");
		return (Expression) result;
	}

	private <T> Pair<T, T> createPairFromSingle(T st) {
		return Pair.of(st, st);
	}

	@Override
	public Pair<Statement, Statement> visitIf_stmt(If_stmtContext ctx) {

		Statement booleanGuard = checkAndExtractSingleStatement(visitTest(ctx.test(0)));
		currentCFG.addNodeIfNotPresent(booleanGuard);

		// Created if exit node
		NoOp ifExitNode = new NoOp(currentCFG, getLocation(ctx));

		// Visit if true block
		Pair<Statement, Statement> trueBlock = visitSuite(ctx.suite(0));

		Statement exitStatementTrueBranch = trueBlock.getRight();
		Statement entryStatementTrueBranch = trueBlock.getLeft();

		currentCFG.addEdge(new TrueEdge(booleanGuard, entryStatementTrueBranch));
		if (!exitStatementTrueBranch.stopsExecution()) {
			currentCFG.addNodeIfNotPresent(ifExitNode);
			currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranch, ifExitNode));
		}

		int testLenght = ctx.test().size();

		Statement lastBooleanGuardElif = booleanGuard;

		// if testLenght is >1 the context contains elif
		if (testLenght > 1) {

			int i = 1;

			// visit all the elif
			while (i < testLenght) {
				Statement booleanGuardElif = checkAndExtractSingleStatement(visitTest(ctx.test(i)));
				currentCFG.addNodeIfNotPresent(booleanGuardElif);
				currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, booleanGuardElif));
				lastBooleanGuardElif = booleanGuardElif;
				Pair<Statement, Statement> trueBlockElif = visitSuite(ctx.suite(i));

				Statement exitStatementTrueBranchElif = trueBlockElif.getRight();
				Statement entryStatementTrueBranchElif = trueBlockElif.getLeft();

				currentCFG.addEdge(new TrueEdge(booleanGuardElif, entryStatementTrueBranchElif));
				if (!exitStatementTrueBranchElif.stopsExecution()) {
					currentCFG.addNodeIfNotPresent(ifExitNode);
					currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranchElif, ifExitNode));
				}
				i = i + 1;
			}
		}
		// If statement with else
		if (ctx.ELSE() != null) {

			Pair<Statement, Statement> falseBlock = visitSuite(ctx.suite(ctx.suite().size() - 1));
			Statement entryStatementFalseBranch = falseBlock.getLeft();
			Statement exitStatementFalseBranch = falseBlock.getRight();

			currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, entryStatementFalseBranch));
			if (!exitStatementFalseBranch.stopsExecution()) {
				currentCFG.addNodeIfNotPresent(ifExitNode);
				currentCFG.addEdge(new SequentialEdge(exitStatementFalseBranch, ifExitNode));
			}
		} else {
			// If statement with no else
			if (!lastBooleanGuardElif.stopsExecution()) {
				currentCFG.addNodeIfNotPresent(ifExitNode);
				currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, ifExitNode));
			}
		}

		return Pair.of(booleanGuard, ifExitNode);
	}

	@Override
	public Pair<Statement, Statement> visitWhile_stmt(While_stmtContext ctx) {

		// create and add exit point of while
		NoOp whileExitNode = new NoOp(currentCFG, getLocation(ctx));
		currentCFG.addNodeIfNotPresent(whileExitNode);

		Statement condition = checkAndExtractSingleStatement(visitTest(ctx.test()));
		currentCFG.addNodeIfNotPresent(condition);

		Pair<Statement, Statement> trueBlock = visitSuite(ctx.suite(0));

		currentCFG.addEdge(new TrueEdge(condition, trueBlock.getLeft()));
		currentCFG.addEdge(new SequentialEdge(trueBlock.getRight(), condition));

		// check if there's an else condition for the while
		if (ctx.ELSE() != null) {
			Pair<Statement, Statement> falseBlock = visitSuite(ctx.suite(1));
			currentCFG.addEdge(new FalseEdge(condition, falseBlock.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseBlock.getRight(), whileExitNode));
		} else {
			currentCFG.addEdge(new FalseEdge(condition, whileExitNode));
		}

		return Pair.of(condition, whileExitNode);
	}

	@Override
	public Pair<Statement, Statement> visitFor_stmt(For_stmtContext ctx) {

		// create and add exit point of for
		NoOp exit = new NoOp(currentCFG, getLocation(ctx));
		currentCFG.addNodeIfNotPresent(exit);

		List<Expression> exprs = extractExpressionsFromExprlist(ctx.exprlist());
		Expression variable;
		if (exprs.size() == 1)
			variable = exprs.get(0);
		else
			variable = new TupleCreation(currentCFG, getLocation(ctx), exprs.toArray(Expression[]::new));
		Expression collection = checkAndExtractSingleExpression(visitTestlist(ctx.testlist()));
		Expression[] collection_pars = { collection };

		VariableRef counter = new VariableRef(
				currentCFG,
				getLocation(ctx),
				"__counter_location" + getLocation(ctx).getLine(), Int32.INSTANCE);
		Expression[] counter_pars = { counter };

		// counter = 0;
		Assignment counter_init = new Assignment(
				currentCFG,
				getLocation(ctx),
				counter,
				new Int32Literal(currentCFG, getLocation(ctx), 0));
		currentCFG.addNodeIfNotPresent(counter_init);

		// counter < collection.size()
		LessThan condition = new LessThan(
				currentCFG,
				getLocation(ctx),
				counter,
				new UnresolvedCall(
						currentCFG,
						getLocation(ctx),
						ASSIGN_STRATEGY,
						MATCHING_STRATEGY,
						TRAVERSAL_STRATEGY,
						CallType.INSTANCE,
						null,
						"size",
						collection_pars));
		currentCFG.addNodeIfNotPresent(condition);

		// element = collection.at(counter)
		Assignment element_assignment = new Assignment(
				currentCFG,
				getLocation(ctx),
				variable,
				new UnresolvedCall(
						currentCFG,
						getLocation(ctx),
						ASSIGN_STRATEGY,
						MATCHING_STRATEGY,
						TRAVERSAL_STRATEGY,
						CallType.INSTANCE,
						null,
						"at",
						counter_pars));
		currentCFG.addNodeIfNotPresent(element_assignment);

		// counter = counter + 1;
		Assignment counter_increment = new Assignment(
				currentCFG,
				getLocation(ctx),
				counter,
				new Addition(
						currentCFG,
						getLocation(ctx),
						counter,
						new Int32Literal(
								currentCFG,
								getLocation(ctx),
								1)));
		currentCFG.addNodeIfNotPresent(counter_increment);

		Pair<Statement, Statement> body = visitSuite(ctx.suite(0));

		currentCFG.addEdge(new SequentialEdge(counter_init, condition));
		currentCFG.addEdge(new TrueEdge(condition, element_assignment));
		currentCFG.addEdge(new SequentialEdge(element_assignment, body.getLeft()));
		currentCFG.addEdge(new SequentialEdge(body.getRight(), counter_increment));
		currentCFG.addEdge(new SequentialEdge(counter_increment, condition));
		currentCFG.addEdge(new FalseEdge(condition, exit));
		return Pair.of(counter_init, exit);
	}

	@Override
	public Pair<Statement, Statement> visitTry_stmt(Try_stmtContext ctx) {
		log.warn("Exceptions are not yet supported. The try block at line " + getLine(ctx) + " of file " + getFilePath()
				+ " is unsoundly translated considering only the code in the try block");
		return visitSuite(ctx.suite(0));
	}

	@Override
	public Pair<Statement, Statement> visitWith_stmt(With_stmtContext ctx) {
		int withSize = ctx.with_item().size();
		Pair<Statement, Statement> withItem = visitWith_item(ctx.with_item(0));
		Pair<Statement, Statement> prev = withItem;
		Pair<Statement, Statement> curr = withItem;
		for (int i = 1; i < withSize; i++) {
			curr = visitWith_item(ctx.with_item(i));
			currentCFG.addEdge(new SequentialEdge(prev.getRight(), curr.getLeft()));
			prev = curr;
		}
		Pair<Statement, Statement> suite = visitSuite(ctx.suite());
		currentCFG.addEdge(new SequentialEdge(curr.getRight(), suite.getLeft()));

		return Pair.of(withItem.getLeft(), suite.getRight());
	}

	@Override
	public Pair<Statement, Statement> visitWith_item(With_itemContext ctx) {
		Statement test = checkAndExtractSingleStatement(visitTest(ctx.test()));
		currentCFG.addNodeIfNotPresent(test);
		if (ctx.expr() != null) {
			Statement expr = checkAndExtractSingleStatement(visitExpr(ctx.expr()));
			currentCFG.addNodeIfNotPresent(expr);
			currentCFG.addEdge(new SequentialEdge(test, expr));
			return Pair.of(test, expr);
		} else {
			return createPairFromSingle(test);
		}

	}

	@Override
	public Pair<Statement, Statement> visitExcept_clause(Except_clauseContext ctx) {
		throw new UnsupportedStatementException();
	}

	private <T> Pair<Statement, Statement> visitListOf(List<T> elements,
			Function<T, Pair<Statement, Statement>> visitor) {
		Statement result = null;
		Statement first = null;
		for (T element : elements) {
			Statement previous = result;
			Pair<Statement, Statement> parsed = visitor.apply(element);
			try {
				if (previous != null)
					currentCFG.addEdge(new SequentialEdge(previous, parsed.getLeft()));
			} catch (UnsupportedStatementException e) {
				// The node was not yet added. Since this method is called from
				// different contexts,
				// sometimes it parses expressions and those are not added as
				// node to the CFG.
				currentCFG.addNodeIfNotPresent(parsed.getLeft());
				currentCFG.addNodeIfNotPresent(previous);
				currentCFG.addEdge(new SequentialEdge(previous, parsed.getLeft()));
			}
			result = parsed.getRight();
			if (first == null)
				first = parsed.getLeft();
		}
		return Pair.of(first, result);
	}

	private Pair<Statement, Statement> visitListOfStatements(List<StmtContext> statements) {
		return visitListOf(statements, this::visitStmt);
	}

	private Pair<Statement, Statement> visitListOfSmallStatements(List<Small_stmtContext> statements) {
		return visitListOf(statements, this::visitSmall_stmt);
	}

	public Pair<Statement, Statement> visitTestlistStarExpr(Expr_stmtContext ctx) {
		// for(Testlist_star_exprContext tstlist : ctx.testlist_star_expr())
		// visitTestlist_star_expr(tstlist);
		return visitListOf(ctx.testlist_star_expr(), this::visitTestlist_star_expr);
	}

	@Override
	public Pair<Statement, Statement> visitTestlist(TestlistContext ctx) {
		return visitListOf(ctx.test(), this::visitTest);
	}

	@Override
	public Pair<Statement, Statement> visitExprlist(ExprlistContext ctx) {
		if (ctx.expr().size() >= 1 && ctx.star_expr().size() >= 1) {
			Pair<Statement, Statement> expPair = visitListOf(ctx.expr(), this::visitExpr);
			Pair<Statement, Statement> starExpPair = visitListOf(ctx.star_expr(), this::visitStar_expr);
			return Pair.of(expPair.getLeft(), starExpPair.getRight());
		} else if (ctx.expr().size() >= 1)
			return visitListOf(ctx.expr(), this::visitExpr);
		else if (ctx.star_expr().size() >= 1)
			return visitListOf(ctx.star_expr(), this::visitStar_expr);
		else
			throw new UnsupportedStatementException("We need to have at least expressions or start expressions");
	}

	@Override
	public Pair<Statement, Statement> visitSuite(SuiteContext ctx) {
		if (ctx.simple_stmt() != null)
			return visitSimple_stmt(ctx.simple_stmt());
		else
			return visitListOfStatements(ctx.stmt());
	}

	@Override
	public Pair<Statement, Statement> visitTest(TestContext ctx) {
		// no if into the condition
		if (ctx.IF() != null) {
			// visit the if into the condition
			Statement trueCase = checkAndExtractSingleStatement(visitOr_test(ctx.or_test(0)));
			Statement booleanGuard = checkAndExtractSingleStatement(visitOr_test(ctx.or_test(1)));
			Pair<Statement, Statement> falseCase = visitTest(ctx.test());
			NoOp testExitNode = new NoOp(currentCFG, getLocation(ctx));

			currentCFG.addNodeIfNotPresent(booleanGuard);
			currentCFG.addNodeIfNotPresent(trueCase);
			currentCFG.addNodeIfNotPresent(falseCase.getLeft());
			currentCFG.addNodeIfNotPresent(falseCase.getRight());
			currentCFG.addNodeIfNotPresent(testExitNode);

			currentCFG.addEdge(new TrueEdge(booleanGuard, trueCase));
			currentCFG.addEdge(new FalseEdge(booleanGuard, falseCase.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseCase.getRight(), testExitNode));
			currentCFG.addEdge(new SequentialEdge(trueCase, testExitNode));

			return Pair.of(booleanGuard, testExitNode);
		} else if (ctx.lambdef() != null) {
			List<Expression> args = extractExpressionsFromVarArgList(ctx.lambdef().varargslist());
			Expression body = checkAndExtractSingleExpression(visitTest(ctx.lambdef().test()));
			return createPairFromSingle(
					new LambdaExpression(
							args,
							body,
							currentCFG,
							getLocation(ctx)));
		} else
			return visitOr_test(ctx.or_test(0));
	}

	@Override
	public Pair<Statement, Statement> visitTest_nocond(Test_nocondContext ctx) {
		if (ctx.or_test() != null) {
			return visitOr_test(ctx.or_test());
		} else {
			return visitLambdef_nocond(ctx.lambdef_nocond());
		}
	}

	@Override
	public Pair<Statement, Statement> visitLambdef(LambdefContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitLambdef_nocond(Lambdef_nocondContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitOr_test(Or_testContext ctx) {

		int nAndTest = ctx.and_test().size();
		if (nAndTest == 1) {
			return visitAnd_test(ctx.and_test(0));
		} else if (nAndTest == 2) {
			return createPairFromSingle(new Or(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(0))),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(1)))));
		} else {
			Expression temp = new Or(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(nAndTest - 2))),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(nAndTest - 1))));
			nAndTest = nAndTest - 2;
			while (nAndTest > 0) {
				temp = new Or(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(--nAndTest))),
						temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitAnd_test(And_testContext ctx) {

		int nNotTest = ctx.not_test().size();
		if (nNotTest == 1) {
			return visitNot_test(ctx.not_test(0));
		} else if (nNotTest == 2) {
			return createPairFromSingle(new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(0))),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(1)))));
		} else {
			Expression temp = new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(nNotTest - 2))),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(nNotTest - 1))));
			nNotTest = nNotTest - 2;
			while (nNotTest > 0) {
				temp = new And(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitNot_test(ctx.not_test(--nNotTest))),
						temp);
			}

			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitNot_test(Not_testContext ctx) {

		if (ctx.NOT() != null) {
			return createPairFromSingle(new Not(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test()))));
		} else {
			return visitComparison(ctx.comparison());
		}

	}

	@Override
	public Pair<Statement, Statement> visitComparison(ComparisonContext ctx) {

		int nExpr = ctx.expr().size();
		Expression result = null;
		switch (nExpr) {
		case 1:
			result = checkAndExtractSingleExpression(visitExpr(ctx.expr(0)));
			break;
		case 2:

			Comp_opContext operator = ctx.comp_op(0);
			Expression left = checkAndExtractSingleExpression(visitExpr(ctx.expr(0)));
			Expression right = checkAndExtractSingleExpression(visitExpr(ctx.expr(1)));
			if (operator.EQUALS() != null)
				result = new Equals(currentCFG, getLocation(ctx), left, right);

			// Python greater (>)
			if (operator.GREATER_THAN() != null) {
				result = new GreaterThan(currentCFG, getLocation(ctx), left, right);
			}
			// Python greater equal (>=)
			if (operator.GT_EQ() != null)
				result = new Equals(currentCFG, getLocation(ctx), left, right);

			// Python in (in)
			if (operator.IN() != null)
				result = new PyIn(currentCFG, getLocation(ctx), left, right);

			// Python is (is)
			if (operator.IS() != null)
				result = new PyIs(currentCFG, getLocation(ctx), left, right);

			// Python less (<)
			if (operator.LESS_THAN() != null)
				result = new LessThan(currentCFG, getLocation(ctx), left, right);

			// Python less equal (<=)
			if (operator.LT_EQ() != null)
				result = new LessOrEqual(currentCFG, getLocation(ctx), left, right);

			// Python not (not)
			if (operator.NOT() != null)
				result = new Not(currentCFG, getLocation(ctx), left);

			// Python not equals (<>)
			if (operator.NOT_EQ_1() != null)
				result = new NotEqual(currentCFG, getLocation(ctx), left, right);

			// Python not equals (!=)
			if (operator.NOT_EQ_2() != null)
				result = new NotEqual(currentCFG, getLocation(ctx), left, right);

			break;
		}

		return createPairFromSingle(result);
	}

	@Override
	public Pair<Statement, Statement> visitComp_op(Comp_opContext ctx) {

		return super.visitComp_op(ctx);
	}

	@Override
	public Pair<Statement, Statement> visitStar_expr(Star_exprContext ctx) {
		throw new UnsupportedStatementException();
	}

	public Pair<Statement, Statement> visitExpr(ExprContext ctx) {
		int nXor = ctx.xor_expr().size();
		if (nXor == 1) {
			// only one Xor
			return visitXor_expr(ctx.xor_expr(0));
		} else if (nXor == 2) {
			// two Xor
			return createPairFromSingle(new PyXor(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(0))),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(1)))));
		} else {
			Expression temp = new PyXor(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(nXor - 2))),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(nXor - 1))));
			nXor = nXor - 2;
			// concatenate all the Xor expressions together
			while (nXor > 0) {
				temp = new PyXor(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(--nXor))),
						temp);
			}
			return createPairFromSingle(temp);
		}

	}

	@Override
	public Pair<Statement, Statement> visitXor_expr(Xor_exprContext ctx) {

		int nAnd = ctx.and_expr().size();

		if (nAnd == 1) {
			return visitAnd_expr(ctx.and_expr(0));
		} else if (nAnd == 2) {
			return createPairFromSingle(new PyXor(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(0))),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(1)))));
		} else {
			Expression temp = new PyXor(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(nAnd - 2))),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(nAnd - 1))));
			nAnd = nAnd - 2;
			// concatenate all the And expressions together
			while (nAnd > 0) {
				temp = new PyXor(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(--nAnd))),
						temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitAnd_expr(And_exprContext ctx) {
		int nShift = ctx.left_shift().size();
		if (nShift == 1) {
			return visitLeft_shift(ctx.left_shift(0));
		} else if (nShift == 2) {
			return createPairFromSingle(new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(0))),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(1)))));
		} else {
			Expression temp = new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(nShift - 2))),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(nShift - 1))));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new And(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(--nShift))),
						temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitLeft_shift(Left_shiftContext ctx) {
		int nShift = ctx.left_shift().size() + 1;
		if (nShift == 1) {
			return visitRight_shift(ctx.right_shift());
		} else if (nShift == 2) {
			return createPairFromSingle(new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitRight_shift(ctx.right_shift())),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(0)))));
		} else {
			Expression temp = new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(nShift - 3))),
					checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(nShift - 2))));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new And(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitLeft_shift(ctx.left_shift(--nShift - 1))),
						temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitRight_shift(Right_shiftContext ctx) {
		int nShift = ctx.right_shift().size() + 1;
		if (nShift == 1) {
			return visitArith_expr(ctx.arith_expr());
		} else if (nShift == 2) {
			return createPairFromSingle(new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr())),
					checkAndExtractSingleExpression(visitRight_shift(ctx.right_shift(0)))));
		} else {
			Expression temp = new And(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitRight_shift(ctx.right_shift(nShift - 3))),
					checkAndExtractSingleExpression(visitRight_shift(ctx.right_shift(nShift - 2))));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new And(currentCFG, getLocation(ctx),
						checkAndExtractSingleExpression(visitRight_shift(ctx.right_shift(--nShift - 1))),
						temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitMinus(MinusContext ctx) {
		if (ctx.arith_expr() == null) {
			return visitTerm(ctx.term());
		} else {
			return createPairFromSingle(new Subtraction(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitTerm(ctx.term())),
					checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr()))));
		}
	}

	@Override
	public Pair<Statement, Statement> visitAdd(AddContext ctx) {

		if (ctx.arith_expr() == null) {
			return visitTerm(ctx.term());
		} else {
			return createPairFromSingle(
					new Addition(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitTerm(ctx.term())),
							checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr()))));
		}

	}

	@Override
	public Pair<Statement, Statement> visitArith_expr(Arith_exprContext ctx) {
		// check if there is minus(-) or an add(+)
		if (ctx.minus() != null) {
			return visitMinus(ctx.minus());
		} else if (ctx.add() != null) {
			return visitAdd(ctx.add());
		} else
			return visitTerm(ctx.term());
	}

	@Override
	public Pair<Statement, Statement> visitMul(MulContext ctx) {

		if (ctx.term() == null) {
			return visitFactor(ctx.factor());
		} else {
			return createPairFromSingle(new Multiplication(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitFactor(ctx.factor())),
					checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}

	}

	public Pair<Statement, Statement> visitMat_mul(Mat_mulContext ctx) {

		if (ctx.term() == null) {
			return visitFactor(ctx.factor());
		} else {
			return createPairFromSingle(new PyMatMul(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitFactor(ctx.factor())),
					checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}

	}

	public Pair<Statement, Statement> visitDiv(DivContext ctx) {

		if (ctx.term() == null) {
			return visitFactor(ctx.factor());
		} else {
			return createPairFromSingle(new Division(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitFactor(ctx.factor())),
					checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}

	}

	public Pair<Statement, Statement> visitMod(ModContext ctx) {

		if (ctx.term() == null) {
			return visitFactor(ctx.factor());
		} else {
			return createPairFromSingle(new Remainder(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitFactor(ctx.factor())),
					checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}

	}

	public Pair<Statement, Statement> visitFloorDiv(FloorDivContext ctx) {

		if (ctx.term() == null) {
			return visitFactor(ctx.factor());
		} else {
			return createPairFromSingle(new PyFloorDiv(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitFactor(ctx.factor())),
					checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}
	}

	@Override
	public Pair<Statement, Statement> visitTerm(TermContext ctx) {
		// check what's the operation in the context
		if (ctx.mul() != null) {
			return visitMul(ctx.mul());
		} else if (ctx.mat_mul() != null) {
			return visitMat_mul(ctx.mat_mul());
		} else if (ctx.div() != null) {
			return visitDiv(ctx.div());
		} else if (ctx.mod() != null) {
			return visitMod(ctx.mod());
		} else if (ctx.floorDiv() != null) {
			return visitFloorDiv(ctx.floorDiv());
		} else if (ctx.factor() != null)
			return visitFactor(ctx.factor());
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitFactor(FactorContext ctx) {
		if (ctx.power() != null) {
			return visitPower(ctx.power());
		}
		return super.visitFactor(ctx);
	}

	@Override
	public Pair<Statement, Statement> visitPower(PowerContext ctx) {

		if (ctx.POWER() != null) {
			return createPairFromSingle(new PyPower(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitAtom_expr(ctx.atom_expr())),
					checkAndExtractSingleExpression(visitFactor(ctx.factor()))));
		} else
			return visitAtom_expr(ctx.atom_expr());
	}

	@Override
	public Pair<Statement, Statement> visitAtom_expr(Atom_exprContext ctx) {
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
			Expression access = checkAndExtractSingleExpression(visitAtom(ctx.atom()));
			String last_name = access instanceof VariableRef ? ((VariableRef) access).getName() : null;
			Expression previous_access = null;
			for (TrailerContext expr : ctx.trailer()) {
				if (expr.NAME() != null) {
					last_name = expr.NAME().getSymbol().getText();
					Global fieldName = new Global(getLocation(ctx), last_name);
					previous_access = access;
					access = new AccessInstanceGlobal(currentCFG, getLocation(expr), access, fieldName);
				} else if (expr.OPEN_PAREN() != null) {
					if (last_name == null)
						throw new UnsupportedStatementException(
								"When invoking a method we need to have always the name before the parentheses");
					List<Expression> pars = new ArrayList<>();
					String method_name = last_name;
					boolean instance = access instanceof AccessInstanceGlobal;
					if (instance)
						pars.add(previous_access);
					if (expr.arglist() != null)
						for (ArgumentContext arg : expr.arglist().argument())
							pars.add(checkAndExtractSingleExpression(visitArgument(arg)));

					access = new UnresolvedCall(
							currentCFG,
							getLocation(expr),
							ASSIGN_STRATEGY,
							MATCHING_STRATEGY,
							TRAVERSAL_STRATEGY,
							instance ? CallType.UNKNOWN : CallType.STATIC,
							null,
							method_name,
							pars.toArray(Expression[]::new));
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
			return createPairFromSingle(access);
		} else
			return visitAtom(ctx.atom());
	}

	@Override
	public Pair<Statement, Statement> visitAtom(AtomContext ctx) {

		if (ctx.NAME() != null) {
			// crete a variable
			return createPairFromSingle(new VariableRef(currentCFG, getLocation(ctx), ctx.NAME().getText()));
		} else if (ctx.NUMBER() != null) {
			String text = ctx.NUMBER().getText().toLowerCase();
			if (text.contains("j"))
				// complex number
				throw new UnsupportedStatementException(
						"complex numbeer are not supported (at " + getLocation(ctx) + ")");

			if (text.contains("e") || text.contains("."))
				// floating point
				return createPairFromSingle(
						new Float32Literal(currentCFG, getLocation(ctx), Float.parseFloat(text)));

			// integer
			return createPairFromSingle(
					new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text)));
		} else if (ctx.FALSE() != null) {
			// create a literal false
			return createPairFromSingle(new FalseLiteral(currentCFG, getLocation(ctx)));
		} else if (ctx.TRUE() != null) {
			// create a literal true
			return createPairFromSingle(new TrueLiteral(currentCFG, getLocation(ctx)));
		} else if (ctx.NONE() != null) {
			// create a literal false
			return createPairFromSingle(new NullLiteral(currentCFG, getLocation(ctx)));
		} else if (ctx.STRING().size() > 0) {
			// create a string
			return createPairFromSingle(new StringLiteral(currentCFG, getLocation(ctx), ctx.STRING(0).getText()));
		} else if (ctx.yield_expr() != null) {
			return visitYield_expr(ctx.yield_expr());
		} else if (ctx.OPEN_BRACE() == null && ctx.dictorsetmaker() != null) {
			return visitDictorsetmaker(ctx.dictorsetmaker());
		} else if (ctx.OPEN_BRACK() != null) {
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			ListCreation r = new ListCreation(currentCFG, getLocation(ctx), sts.toArray(Expression[]::new));
			return createPairFromSingle(r);
		} else if (ctx.OPEN_PAREN() != null) {
			if (ctx.yield_expr() != null)
				throw new UnsupportedStatementException("yield expressions not supported");
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			TupleCreation r = new TupleCreation(currentCFG, getLocation(ctx), sts.toArray(Expression[]::new));
			return createPairFromSingle(r);
		} else if (ctx.OPEN_BRACE() != null) {
			List<Pair<Expression, Expression>> values = extractPairsFromDictorSet(ctx.dictorsetmaker());
			@SuppressWarnings("unchecked")
			DictionaryCreation r = new DictionaryCreation(currentCFG, getLocation(ctx),
					values.toArray(Pair[]::new));
			return createPairFromSingle(r);
		}
		throw new UnsupportedStatementException();
	}

	private List<Pair<Expression, Expression>> extractPairsFromDictorSet(DictorsetmakerContext ctx) {
		if (ctx == null)
			return new ArrayList<>();
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (ctx.test().size() != 2 * ctx.COLON().size())
			throw new UnsupportedStatementException(
					"We support only initialization of dictonaries in the form of <key> : <value>");
		for (int i = 0; i < ctx.COLON().size(); i++) {
			Expression left = checkAndExtractSingleExpression(visitTest(ctx.test(2 * i)));
			Expression right = checkAndExtractSingleExpression(visitTest(ctx.test(2 * i + 1)));
			result.add(Pair.of(left, right));
		}
		return result;
	}

	private List<Expression> extractExpressionsFromTestlist(TestlistContext ctx) {
		return extractExpressionsFromListOfTests(ctx.test());
	}

	private List<Expression> extractExpressionsFromExprlist(ExprlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx.expr().size() == 0)
			return result;
		for (ExprContext e : ctx.expr())
			result.add(checkAndExtractSingleExpression(visitExpr(e)));
		return result;
	}

	private List<Expression> extractExpressionsFromVarArgList(VarargslistContext ctx) {
		return extractExpressionsFromListOfTests(ctx.test());
	}

	private List<Expression> extractExpressionsFromYieldArg(Yield_argContext ctx) {
		if (ctx.test() != null) {
			List<Expression> r = new ArrayList<>();
			r.add(checkAndExtractSingleExpression(visitTest(ctx.test())));
			return r;
		} else
			return extractExpressionsFromTestlist(ctx.testlist());
	}

	private List<Expression> extractExpressionsFromListOfTests(List<TestContext> l) {
		List<Expression> result = new ArrayList<>();
		if (l.size() == 0)
			return result;
		for (TestContext e : l)
			result.add(checkAndExtractSingleExpression(visitTest(e)));
		return result;
	}

	private List<Expression> extractExpressionsFromSubscriptlist(SubscriptlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx.subscript_().size() == 0)
			return result;
		for (Subscript_Context e : ctx.subscript_())
			result.add(checkAndExtractSingleExpression(visitSubscript_(e)));
		return result;
	}

	private List<Expression> extractExpressionsFromTestlist_comp(Testlist_compContext ctx) {
		List<Expression> result = new ArrayList<>();
		if (ctx == null || ctx.testOrStar() == null || ctx.testOrStar().size() == 0)
			return result;
		for (TestOrStarContext e : ctx.testOrStar())
			result.add(checkAndExtractSingleExpression(visitTestOrStar(e)));
		return result;
	}

	@Override
	public Pair<Statement, Statement> visitTestlist_comp(Testlist_compContext ctx) {
		Pair<Statement, Statement> elem = visitTestOrStar(ctx.testOrStar(0));
		Pair<Statement, Statement> prev;
		Pair<Statement, Statement> current;
		if (ctx.comp_for() != null) {
			Pair<Statement, Statement> forClause = visitComp_for(ctx.comp_for());
			currentCFG.addEdge(new SequentialEdge(elem.getRight(), forClause.getLeft()));
			int nTestOrStar = ctx.testOrStar().size();
			prev = forClause;
			for (int i = 1; i < nTestOrStar; i++) {
				current = visitTestOrStar(ctx.testOrStar(i));
				currentCFG.addEdge(new SequentialEdge(prev.getRight(), current.getLeft()));
				prev = current;
			}
			return Pair.of(elem.getLeft(), prev.getRight());
		} else
			return elem;
	}

	@Override
	public Pair<Statement, Statement> visitTrailer(TrailerContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitSubscriptlist(SubscriptlistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitSubscript_(Subscript_Context ctx) {
		if (ctx.COLON() != null) {
			SourceCodeLocation loc = getLocation(ctx);
			Expression left = ctx.test1() == null ? new Empty(currentCFG, loc)
					: checkAndExtractSingleExpression(visitTest(ctx.test1().test()));
			Expression middle = ctx.test2() == null ? new Empty(currentCFG, loc)
					: checkAndExtractSingleExpression(visitTest(ctx.test2().test()));
			Expression right = ctx.sliceop() == null || ctx.sliceop().test() == null ? new Empty(currentCFG, loc)
					: checkAndExtractSingleExpression(visitTest(ctx.sliceop().test()));
			return createPairFromSingle(new RangeValue(currentCFG, loc, left, middle, right));
		} else
			return visitTest(ctx.test());
	}

	@Override
	public Pair<Statement, Statement> visitSliceop(SliceopContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDictorsetmaker(DictorsetmakerContext ctx) {
		if (ctx.COLON().size() == 0) {
			List<Expression> values = new ArrayList<>();
			for (TestContext exp : ctx.test())
				values.add(checkAndExtractSingleExpression(visitTest(exp)));
			return createPairFromSingle(
					new SetCreation(currentCFG, getLocation(ctx), values.toArray(Expression[]::new)));
		} else {
			throw new UnsupportedStatementException();
		}
	}

	@Override
	public Pair<Statement, Statement> visitClassdef(ClassdefContext ctx) {
		Unit previous = this.currentUnit;
		String name = ctx.NAME().getSymbol().getText();
		this.currentUnit = new PythonUnit(new SourceCodeLocation(name, 0, 0), name, true);
		parseClassBody(ctx.suite());
		this.currentUnit = previous;
		return null;
	}

	private void parseClassBody(SuiteContext ctx) {
		List<Pair<VariableRef, Expression>> fields_init = new ArrayList<>();
		if (ctx.simple_stmt() != null)
			throw new UnsupportedStatementException(
					"Inside the body of a class we should have only field and method definitions");
		for (StmtContext stmt : ctx.stmt()) {
			if (stmt.simple_stmt() != null) {
				Pair<VariableRef, Expression> p = parseField(stmt.simple_stmt());
				currentUnit.addGlobal(new Global(getLocation(ctx), p.getLeft().getName()));
				if (p.getRight() != null)
					fields_init.add(p);
			} else if (stmt.compound_stmt().funcdef() != null) {
				this.visitFuncdef(stmt.compound_stmt().funcdef());
			} else if (stmt.compound_stmt().decorated() != null) {
				log.warn("Ignoring decorator " + stmt.compound_stmt().decorated().decorators().getText()
						+ " at code location " + getLocation(stmt));
				DecoratedContext c = stmt.compound_stmt().decorated();
				if (c.funcdef() != null)
					this.visitFuncdef(c.funcdef());
				else if (c.classdef() != null)
					this.visitClassdef(c.classdef());
				else
					throw new UnsupportedStatementException("We support only decorated classes and methods");
			} else
				throw new UnsupportedStatementException(
						"Inside the body of a class we should have only field and method definitions");
		}
		dumpConstructor(fields_init, getLocation(ctx));
	}

	private void dumpConstructor(List<Pair<VariableRef, Expression>> fields_init, CodeLocation location) {
		if (fields_init.size() > 0) {
			PyCFG oldCFG = currentCFG;
			currentCFG = new PyCFG(new CFGDescriptor(location, currentUnit, true, "<init>"));
			program.addCFG(currentCFG);
			Statement previous = null;
			for (Pair<VariableRef, Expression> init : fields_init) {
				Statement f = new Assignment(currentCFG, init.getLeft().getLocation(), init.getLeft(), init.getRight());
				currentCFG.addNodeIfNotPresent(f, previous == null);
				if (previous != null)
					currentCFG.addEdge(new SequentialEdge(previous, f));
				previous = f;
			}
			currentUnit.addCFG(currentCFG);
			currentCFG = oldCFG;
		}

	}

	private Pair<VariableRef, Expression> parseField(Simple_stmtContext st) {
		Statement result = checkAndExtractSingleStatement(visitSimple_stmt(st));
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
	public Pair<Statement, Statement> visitArglist(ArglistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitArgument(ArgumentContext ctx) {
		if (ctx.ASSIGN() != null) {
			return createPairFromSingle(createAssign(visitTest(ctx.test(0)), visitTest(ctx.test(1)), getLocation(ctx)));
		} else if (ctx.STAR() != null)
			return createPairFromSingle(new StarExpression(currentCFG, getLocation(ctx),
					checkAndExtractSingleExpression(visitTest(ctx.test(0)))));
		else if (ctx.comp_for() != null || ctx.POWER() != null || ctx.test().size() != 1)
			throw new UnsupportedStatementException("We support only simple arguments in method calls");
		else
			return visitTest(ctx.test(0));
	}

	private PyAssign createAssign(Pair<Statement, Statement> left, Pair<Statement, Statement> right,
			CodeLocation loc) {
		Statement target = checkAndExtractSingleStatement(left);
		Statement expression = checkAndExtractSingleStatement(right);
		if ((!(target instanceof Expression)) || (!(expression instanceof Expression)))
			throw new UnsupportedStatementException(
					"Assignments require expression both in the left and in the right hand side");
		return new PyAssign(currentCFG, loc, (Expression) target, (Expression) expression);
	}

	@Override
	public Pair<Statement, Statement> visitComp_iter(Comp_iterContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitComp_for(Comp_forContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitComp_if(Comp_ifContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitEncoding_decl(Encoding_declContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitYield_expr(Yield_exprContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitYield_arg(Yield_argContext ctx) {
		throw new UnsupportedStatementException();
	}

	private static void setupProgram(Program program) {
		program.registerType(PyListType.INSTANCE);
		program.registerType(BoolType.INSTANCE);
		program.registerType(StringType.INSTANCE);
		program.registerType(Int32.INSTANCE);
		program.registerType(Float32.INSTANCE);
		program.registerType(NullType.INSTANCE);

		for (CompilationUnit lib : LibrarySpecificationProvider.getLibraryUnits()) {
			PyLibraryType.addUnit(lib);
			program.addCompilationUnit(lib);
			program.registerType(new PyLibraryType(lib.getName()));
		}

		LibrarySpecificationProvider.getAllStandardLibraryMethods(program).forEach(program::addConstruct);

		for (CFG cfg : program.getCFGs())
			if (cfg.getDescriptor().getName().equals("main"))
				program.addEntryPoint(cfg);
	}
}