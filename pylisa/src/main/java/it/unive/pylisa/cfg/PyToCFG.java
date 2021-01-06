package it.unive.pylisa.cfg;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;


import it.unive.lisa.AnalysisException;
import it.unive.lisa.LiSA;
import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.CFGDescriptor;
import it.unive.lisa.cfg.Parameter;
import it.unive.lisa.cfg.edge.FalseEdge;
import it.unive.lisa.cfg.edge.SequentialEdge;
import it.unive.lisa.cfg.edge.TrueEdge;
import it.unive.lisa.cfg.statement.Assignment;
import it.unive.lisa.cfg.statement.Expression;
import it.unive.lisa.cfg.statement.Literal;
import it.unive.lisa.cfg.statement.NoOp;
import it.unive.lisa.cfg.statement.Statement;
import it.unive.lisa.cfg.statement.Variable;
import it.unive.lisa.logging.IterationLogger;
import it.unive.pylisa.antlr.Python3BaseVisitor;
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
import it.unive.pylisa.antlr.Python3Parser.Shift_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Simple_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Single_inputContext;
import it.unive.pylisa.antlr.Python3Parser.SliceopContext;
import it.unive.pylisa.antlr.Python3Parser.Small_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Star_exprContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.SubscriptContext;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.SuiteContext;
import it.unive.pylisa.antlr.Python3Parser.TermContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
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
import it.unive.pylisa.cfg.expression.binary.PyAdd;
import it.unive.pylisa.cfg.expression.binary.PyAnd;
import it.unive.pylisa.cfg.expression.binary.PyDiv;
import it.unive.pylisa.cfg.expression.binary.PyEquals;
import it.unive.pylisa.cfg.expression.binary.PyFloorDiv;
import it.unive.pylisa.cfg.expression.binary.PyGreater;
import it.unive.pylisa.cfg.expression.binary.PyIn;
import it.unive.pylisa.cfg.expression.binary.PyIs;
import it.unive.pylisa.cfg.expression.binary.PyLess;
import it.unive.pylisa.cfg.expression.binary.PyLessEqual;
import it.unive.pylisa.cfg.expression.binary.PyMatMul;
import it.unive.pylisa.cfg.expression.binary.PyMinus;
import it.unive.pylisa.cfg.expression.binary.PyMod;
import it.unive.pylisa.cfg.expression.binary.PyMul;
import it.unive.pylisa.cfg.expression.binary.PyNot1;
import it.unive.pylisa.cfg.expression.binary.PyNot2;
import it.unive.pylisa.cfg.expression.binary.PyOr;
import it.unive.pylisa.cfg.expression.binary.PyShiftLeft;
import it.unive.pylisa.cfg.expression.binary.PyShiftRight;
import it.unive.pylisa.cfg.expression.binary.PyXor;
import it.unive.pylisa.cfg.expression.unary.PyNot;
import it.unive.pylisa.cfg.type.PyIntType;
import it.unive.pylisa.cfg.type.PyStringType;

public class PyToCFG<T> extends Python3BaseVisitor<T> {

	private static final Logger log = LogManager.getLogger(PyToCFG.class);

	/**
	 * Python program file path.
	 */
	private final String filePath;

	/**
	 * Collection of CFGs collected into the Python program at filePath.
	 */
	private Collection<CFG> cfgs;

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 * 
	 * @param filePath file path to a Python program.
	 */
	public PyToCFG(String filePath) {
		this.cfgs = new HashSet<CFG>();
		this.filePath = filePath;
	}

	/**
	 * Returns the parsed file path.
	 * 
	 * @return the parsed file path
	 */
	public String getFilePath() {
		return filePath;
	}

	/**
	 * Returns the parsed CFGs
	 * 
	 * @return the parsed CFGs
	 */
	public Collection<CFG> getCFGs() {
		return cfgs;
	}

	/**
	 * Current CFG to parse
	 */
	private CFG currentCFG;

	public static void main(String[] args) throws IOException, AnalysisException {
		String file = "src/test/resources/pyTest/py1.py";
		PyToCFG translator = new PyToCFG(file);
		LiSA lisa = new LiSA();
		Collection<CFG> cfgs = translator.toLiSACFG();
		cfgs.forEach(lisa::addCFG);
		lisa.setDumpCFGs(true);
		lisa.setWorkdir("workdir");
		lisa.run();
	}

	/**
	 * Returns the collection of @CFG in a Python program at filePath.
	 * 
	 * @return collection of @CFG in file
	 * @throws IOException if {@code stream} to file cannot be written to or closed
	 */

	public Collection<CFG> toLiSACFG() throws IOException {
		log.info("PyToCFG setup...");
		log.info("Reading file... " + filePath);

		InputStream stream;
		try {
			stream = new FileInputStream(getFilePath());
		} catch (FileNotFoundException e) {
			System.err.println(filePath + " does not exist. Exiting.");
			return new ArrayList<>();
		}

		Python3Lexer lexer = new Python3Lexer(CharStreams.fromStream(stream, StandardCharsets.UTF_8));
		Python3Parser parser = new Python3Parser(new CommonTokenStream(lexer));
		ParseTree tree = parser.file_input();

		log.info(tree.toStringTree(parser));
		visit(tree);

		stream.close();

		return cfgs;
	}

	@Override
	public T visit(ParseTree tree) {

		if (tree instanceof File_inputContext)
			return visitFile_input((File_inputContext) tree);

		return null;
	}

	@Override
	public T visitSingle_input(Single_inputContext ctx) {
		// TODO Auto-generated method stub
		return super.visitSingle_input(ctx);
	}

	@Override
	public T visitFile_input(File_inputContext ctx) {
		for (StmtContext stmt : IterationLogger.iterate(log, ctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Compound_stmtContext comp = stmt.compound_stmt();
			FuncdefContext funcDecl = comp.funcdef();
			currentCFG = new CFG(buildCFGDescriptor(funcDecl));
			cfgs.add(currentCFG);
			visitFuncdef(funcDecl);
			try {
				Writer w = new FileWriter("./output.txt");
				currentCFG.dump(w, "Prova");
				w.close();
				log.info("Done");
			} catch (IOException e) {
				log.info("c'è stato un errore");
				e.printStackTrace();
			}
			
		}

//		
//		// Visit of each FunctionDeclContext appearing in the source code
//		// and creating the corresponding CFG object (we do this to handle CFG calls)
//		for (FuncdefContext funcDef : IterationLogger.iterate(log, ctx., "Parsing function declarations...", "Function declarations")) 
//			cfgs.add(new CFG(buildCFGDescriptor(funcDef)));
//
//		// Visit of each FunctionDeclContext populating the corresponding cfg
//		for (FunctionDeclContext funcDecl : IterationLogger.iterate(log, ctx.functionDecl(), "Visiting function declarations...", "Function declarations")) {
//			currentCFG = getCFGByName(funcDecl.IDENTIFIER().getText());
//			visitFunctionDecl(funcDecl);			
//		}

		return null;

	}

	private int getLine(ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	private int getCol(ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	private int getCol(Token ctx) {
		return ctx.getCharPositionInLine();
	}

	private int getLine(Token ctx) {
		return ctx.getLine();
	}

	private CFGDescriptor buildCFGDescriptor(FuncdefContext funcDecl) {
		String funcName = funcDecl.NAME().getText();
		TypedargslistContext formalPars = funcDecl.parameters().typedargslist();

		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(funcName, cfgArgs);
	}

	@Override
	public T visitEval_input(Eval_inputContext ctx) {
		// TODO Auto-generated method stub
		return super.visitEval_input(ctx);
	}

	@Override
	public T visitDecorator(DecoratorContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDecorator(ctx);
	}

	@Override
	public T visitDecorators(DecoratorsContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDecorators(ctx);
	}

	@Override
	public T visitDecorated(DecoratedContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDecorated(ctx);
	}

	@Override
	public T visitAsync_funcdef(Async_funcdefContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAsync_funcdef(ctx);
	}

	@Override
	public T visitFuncdef(FuncdefContext ctx) {
		// TODO Auto-generated method stub
		return super.visitFuncdef(ctx);
	}

	@Override
	public T visitParameters(ParametersContext ctx) {
		// TODO Auto-generated method stub
		return super.visitParameters(ctx);
	}

	@Override
	public T visitTypedargslist(TypedargslistContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTypedargslist(ctx);
	}

	@Override
	public T visitTfpdef(TfpdefContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTfpdef(ctx);
	}

	@Override
	public T visitVarargslist(VarargslistContext ctx) {
		// TODO Auto-generated method stub
		return super.visitVarargslist(ctx);
	}

	@Override
	public T visitVfpdef(VfpdefContext ctx) {
		// TODO Auto-generated method stub
		return super.visitVfpdef(ctx);
	}

	@Override
	public T visitStmt(StmtContext ctx) {

		// o simple o compound

		Object result = visitChildren(ctx);
		/*
		 * if (!(result instanceof Pair<?,?>)) throw new
		 * IllegalStateException("Pair of Statements expected"); else
		 */
		return (T) result;
	}

	@Override
	public T visitSimple_stmt(Simple_stmtContext ctx) {
	
			
			Statement simple = (Statement)visitSmall_stmt(ctx.small_stmt(0));
			log.info("sono in simple");
			log.info(simple);
			
			Pair<Statement, Statement> result = Pair.of(simple,simple);
			
			return (T) result;
	}

	@Override
	public T visitSmall_stmt(Small_stmtContext ctx) {
		// TODO Auto-generated method stub

		T temp = visitChildren(ctx);
		log.info("small");
		log.info(temp);
		return temp;
	}

	@Override
	public T visitExpr_stmt(Expr_stmtContext ctx) {
		// TODO Auto-generated method stub

		T assegnazione = null;
		log.info("passo per di qui");
		log.info(ctx.ASSIGN().size());
		
		// è un assegnazione
		if (ctx.ASSIGN().size() > 0) {
			T target = visitChildren(ctx.testlist_star_expr(0));
			T expression = visitChildren(ctx.testlist_star_expr(1));
			log.info(target);
			log.info(expression);
			assegnazione = (T) new Assignment(currentCFG, (Expression) target, (Expression) expression);
			log.info(assegnazione);
			currentCFG.addNode((Statement) assegnazione);
		}
		log.info(assegnazione);
		return (T)assegnazione;
	}

	@Override
	public T visitAnnassign(AnnassignContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAnnassign(ctx);
	}

	@Override
	public T visitTestlist_star_expr(Testlist_star_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTestlist_star_expr(ctx);
	}

	@Override
	public T visitAugassign(AugassignContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAugassign(ctx);
	}

	@Override
	public T visitDel_stmt(Del_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDel_stmt(ctx);
	}

	@Override
	public T visitPass_stmt(Pass_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitPass_stmt(ctx);
	}

	@Override
	public T visitFlow_stmt(Flow_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitFlow_stmt(ctx);
	}

	@Override
	public T visitBreak_stmt(Break_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitBreak_stmt(ctx);
	}

	@Override
	public T visitContinue_stmt(Continue_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitContinue_stmt(ctx);
	}

	@Override
	public T visitReturn_stmt(Return_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitReturn_stmt(ctx);
	}

	@Override
	public T visitYield_stmt(Yield_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitYield_stmt(ctx);
	}

	@Override
	public T visitRaise_stmt(Raise_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitRaise_stmt(ctx);
	}

	@Override
	public T visitImport_stmt(Import_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitImport_stmt(ctx);
	}

	@Override
	public T visitImport_name(Import_nameContext ctx) {
		// TODO Auto-generated method stub
		return super.visitImport_name(ctx);
	}

	@Override
	public T visitImport_from(Import_fromContext ctx) {
		// TODO Auto-generated method stub
		return super.visitImport_from(ctx);
	}

	@Override
	public T visitImport_as_name(Import_as_nameContext ctx) {
		// TODO Auto-generated method stub
		return super.visitImport_as_name(ctx);
	}

	@Override
	public T visitDotted_as_name(Dotted_as_nameContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDotted_as_name(ctx);
	}

	@Override
	public T visitImport_as_names(Import_as_namesContext ctx) {
		// TODO Auto-generated method stub
		return super.visitImport_as_names(ctx);
	}

	@Override
	public T visitDotted_as_names(Dotted_as_namesContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDotted_as_names(ctx);
	}

	@Override
	public T visitDotted_name(Dotted_nameContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDotted_name(ctx);
	}

	@Override
	public T visitGlobal_stmt(Global_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitGlobal_stmt(ctx);
	}

	@Override
	public T visitNonlocal_stmt(Nonlocal_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitNonlocal_stmt(ctx);
	}

	@Override
	public T visitAssert_stmt(Assert_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAssert_stmt(ctx);
	}

	@Override
	public T visitCompound_stmt(Compound_stmtContext ctx) {
		// TODO Auto-generated method stub
		return (T) visitChildren(ctx);
	}

	@Override
	public T visitAsync_stmt(Async_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAsync_stmt(ctx);
	}

	@Override
	public T visitIf_stmt(If_stmtContext ctx) {

		Statement booleanGuard = (Statement) visitTest(ctx.test(0));
		currentCFG.addNode(booleanGuard);

		NoOp ifExitNode = new NoOp(currentCFG);
		currentCFG.addNode(ifExitNode);

		Pair<Statement, Statement> trueBlock = (Pair<Statement, Statement>) visitSuite(ctx.suite(0));
		
		
		Statement exitStatementTrueBranch = trueBlock.getRight();
		Statement entryStatementTrueBranch = trueBlock.getLeft();

		currentCFG.addNode(exitStatementTrueBranch);
		currentCFG.addNode(entryStatementTrueBranch);
		
		
		// if testLenght is >1 the context contains elif
		

		
		currentCFG.addEdge(new TrueEdge(booleanGuard, entryStatementTrueBranch));
		currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranch, ifExitNode));

		
		int testLenght = ctx.test().size();
		log.info("test lenght");
		
		log.info(testLenght);
		
		Statement lastBooleanGuardElif = booleanGuard;
		//ho almeno un elif
		if(testLenght>1) {
			// If statement without else but with elif
				int i = 1;
				// visit every elif
				
				while (i < testLenght) {
					Statement booleanGuardElif = (Statement) visitTest(ctx.test(i));
					currentCFG.addNode(booleanGuardElif);
					currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, booleanGuardElif));
					lastBooleanGuardElif = booleanGuardElif;
					Pair<Statement, Statement> trueBlockElif = (Pair<Statement, Statement>) visitSuite(ctx.suite(i));
					
					Statement exitStatementTrueBranchElif = trueBlockElif.getRight();
					Statement entryStatementTrueBranchElif = trueBlockElif.getLeft();
					
					currentCFG.addNode(exitStatementTrueBranchElif);
					currentCFG.addNode(entryStatementTrueBranchElif);
					
					currentCFG.addEdge(new TrueEdge(booleanGuardElif, entryStatementTrueBranchElif));
					currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranchElif, ifExitNode));
					i=i+1;
				}
		}
				if (ctx.ELSE() !=null) {
					// If statement with else
		
					Pair<Statement, Statement> falseBlock = (Pair<Statement, Statement>) visitSuite(ctx.suite(ctx.suite().size()-1));
					Statement entryStatementFalseBranch = falseBlock.getLeft();
					Statement exitStatementFalseBranch = falseBlock.getRight();
					currentCFG.addNode(entryStatementFalseBranch);
					currentCFG.addNode(exitStatementFalseBranch);
					
					currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, entryStatementFalseBranch));
					currentCFG.addEdge(new SequentialEdge(exitStatementFalseBranch, ifExitNode));
				}else {
					// non esiste else quindi o termino if o termino elif
					currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, ifExitNode));
				}
		


		return (T) Pair.of(booleanGuard,ifExitNode);
	}

	@Override
	public T visitWhile_stmt(While_stmtContext ctx) {
		
		NoOp whileExitNode = new NoOp(currentCFG);
		currentCFG.addNode(whileExitNode);
		
		Statement condition  = (Statement) visitTest(ctx.test());
		currentCFG.addNode(condition);

		Pair<Statement, Statement> trueBlock = (Pair<Statement, Statement>) visitSuite(ctx.suite(0));
		
		currentCFG.addEdge(new TrueEdge(condition, trueBlock.getLeft()));
		currentCFG.addEdge(new SequentialEdge(trueBlock.getRight(), condition));
		
		if(ctx.ELSE()!=null) {
			Pair<Statement, Statement> falseBlock = (Pair<Statement, Statement>) visitSuite(ctx.suite(1));
			currentCFG.addEdge(new FalseEdge(condition, falseBlock.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseBlock.getRight(), whileExitNode));
		}else {
			currentCFG.addEdge(new FalseEdge(condition, whileExitNode));
		}
		
		
		return (T) Pair.of(condition, whileExitNode);
	}

	@Override
	public T visitFor_stmt(For_stmtContext ctx) {
		
		Pair<Statement, Statement> exprlist=(Pair<Statement, Statement>) visitExprlist(ctx.exprlist());
		
		Pair<Statement, Statement> testList=(Pair<Statement, Statement>) visitTestlist(ctx.testlist());
		currentCFG.addNode(testList.getLeft());
		currentCFG.addNode(testList.getRight());
		
		Pair<Statement, Statement> body=(Pair<Statement, Statement>) visitSuite(ctx.suite(0));
		if(ctx.ELSE()!=null) {
			Pair<Statement, Statement> falseCond=(Pair<Statement, Statement>) visitSuite(ctx.suite(1));
		}else {
			
		}
		
		return super.visitFor_stmt(ctx);
	}

	@Override
	public T visitTry_stmt(Try_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTry_stmt(ctx);
	}

	@Override
	public T visitWith_stmt(With_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitWith_stmt(ctx);
	}

	@Override
	public T visitWith_item(With_itemContext ctx) {
		// TODO Auto-generated method stub
		return super.visitWith_item(ctx);
	}

	@Override
	public T visitExcept_clause(Except_clauseContext ctx) {
		// TODO Auto-generated method stub
		return super.visitExcept_clause(ctx);
	}

	@Override
	public T visitSuite(SuiteContext ctx) {
		if (ctx.simple_stmt() != null) {
			Statement simple = (Statement)visitSimple_stmt(ctx.simple_stmt());
			Pair<Statement, Statement> result = Pair.of(simple,simple);
			log.info("sono dentro al simple");
			log.info(result);
			
			return (T)result;
		} else {
			int nStatement = ctx.stmt().size();
			if (nStatement == 1) {
				Pair<Statement, Statement> stmt = (Pair<Statement, Statement>) visitStmt(ctx.stmt(0));
				return (T) stmt;
			} else{
				Pair<Statement, Statement> firstStmt = (Pair<Statement, Statement>) visitStmt(ctx.stmt(0));
				Pair<Statement, Statement> lastStmt = (Pair<Statement, Statement>) visitStmt(ctx.stmt(1));
				currentCFG.addEdge(new SequentialEdge(firstStmt.getRight(), lastStmt.getLeft()));
				if (nStatement > 2) {
					int i = 1;
					Pair<Statement, Statement> prevStm = firstStmt;
					Pair<Statement, Statement> currentStm = firstStmt;
					while (i < nStatement) {
						currentStm = (Pair<Statement, Statement>) visitStmt(ctx.stmt(i));
						currentCFG.addEdge(new SequentialEdge(prevStm.getRight(), currentStm.getLeft()));
						prevStm = currentStm;
						i = i + 1;
					}

					currentCFG.addEdge(new SequentialEdge(currentStm.getRight(), lastStmt.getLeft()));
				}
				
				return (T)Pair.of(firstStmt.getLeft(), lastStmt.getRight());
			}
		}
	}

	@Override
	public T visitTest(TestContext ctx) {
		if (ctx.IF() == null) {
			return visitOr_test(ctx.or_test(0));
		} else {
			Statement trueCase = (Statement) visitOr_test(ctx.or_test(0));
			Statement booleanGuard = (Statement) visitOr_test(ctx.or_test(1));
			Pair<Statement,Statement> falseCase = (Pair<Statement,Statement>) visitTest(ctx.test());
			NoOp testExitNode = new NoOp(currentCFG);
			
			currentCFG.addEdge(new TrueEdge(booleanGuard, trueCase));
			currentCFG.addEdge(new FalseEdge(booleanGuard, falseCase.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseCase.getRight(), testExitNode));
			currentCFG.addEdge(new SequentialEdge(trueCase, testExitNode));

			return (T) Pair.of(booleanGuard, testExitNode);
		}

	}

	@Override
	public T visitTest_nocond(Test_nocondContext ctx) {
		if (ctx.or_test() != null) {
			return visitOr_test(ctx.or_test());
		} else {
			return visitLambdef_nocond(ctx.lambdef_nocond());
		}
	}

	@Override
	public T visitLambdef(LambdefContext ctx) {
		// TODO Auto-generated method stub
		return super.visitLambdef(ctx);
	}

	@Override
	public T visitLambdef_nocond(Lambdef_nocondContext ctx) {
		// TODO Auto-generated method stub
		return super.visitLambdef_nocond(ctx);
	}

	@Override
	public T visitOr_test(Or_testContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nAndTest = ctx.and_test().size();
		int i = 0;
		if (nAndTest == 1) {
			return visitAnd_test(ctx.and_test(0));
		} else if (nAndTest == 2) {
			return (T) new PyOr(currentCFG, "", line, col, (Expression) visitAnd_test(ctx.and_test(0)),
					(Expression) visitAnd_test(ctx.and_test(1)),"or");
		} else {
			T temp = (T) new PyOr(currentCFG, "", line, col, (Expression) visitAnd_test(ctx.and_test(nAndTest - 2)),
					(Expression) visitAnd_test(ctx.and_test(nAndTest - 1)),"or");
			nAndTest = nAndTest - 2;
			while (nAndTest > 0) {
				temp = (T) new PyOr(currentCFG, "", line, col, (Expression) visitAnd_test(ctx.and_test(--nAndTest)),
						(Expression) temp,"or");
			}
			return temp;
		}
	}

	@Override
	public T visitAnd_test(And_testContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nNotTest = ctx.not_test().size();
		int i = 0;
		if (nNotTest == 1) {
			return visitNot_test(ctx.not_test(0));
		} else if (nNotTest == 2) {
			return (T) new PyAnd(currentCFG, "", line, col, (Expression) visitNot_test(ctx.not_test(0)),
					(Expression) visitNot_test(ctx.not_test(1)));
		} else {
			// nNotTest= 5
			T temp = (T) new PyAnd(currentCFG, "", line, col, (Expression) visitNot_test(ctx.not_test(nNotTest - 2)),
					(Expression) visitNot_test(ctx.not_test(nNotTest - 1)));
			nNotTest = nNotTest - 2;
			while (nNotTest > 0) {
				temp = (T) new PyAnd(currentCFG, "", line, col, (Expression) visitNot_test(ctx.not_test(--nNotTest)),
						(Expression) temp);
			}

			return temp;
		}
	}

	@Override
	public T visitNot_test(Not_testContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		if (ctx.NOT() != null) {
			return (T) new PyNot(currentCFG, "", line, col, (Expression) visitNot_test(ctx.not_test()));
		} else {
			return visitComparison(ctx.comparison());
		}

	}

	@Override
	public T visitComparison(ComparisonContext ctx) {

		int line = getLine(ctx);
		int col = getCol(ctx);

		// ho solo un expression
		int nExpr = ctx.expr().size();
		T result = null;
		switch (nExpr) {
		case 1:
			result = visitExpr(ctx.expr(0));
			break;
		case 2:

			Comp_opContext operator = ctx.comp_op(0);
			Expression left = (Expression) visitExpr(ctx.expr(0));
			Expression right = (Expression) visitExpr(ctx.expr(1));
			if (operator.EQUALS() != null)
				result = (T) new PyEquals(currentCFG, "", line, col, left, right);

			// Python greater (>)
			if (operator.GREATER_THAN() != null) {
				result = (T) new PyGreater(currentCFG, "", line, col, left, right);
			}
			// Python greater equal (>=)
			if (operator.GT_EQ() != null)
				result = (T) new PyEquals(currentCFG, "", line, col, left, right);

			// Python in (in)
			if (operator.IN() != null)
				result = (T) new PyIn(currentCFG, "", line, col, left, right);

			// Python is (is)
			if (operator.IS() != null)
				result = (T) new PyIs(currentCFG, "", line, col, left, right);

			// Python less (<)
			if (operator.LESS_THAN() != null)
				result = (T) new PyLess(currentCFG, "", line, col, left, right);

			// Python less equal (<=)
			if (operator.LT_EQ() != null)
				result = (T) new PyLessEqual(currentCFG, "", line, col, left, right);

			// Python not (not)
			if (operator.NOT() != null)
				result = (T) new PyNot(currentCFG, "", line, col, left);

			// Python not equals (<>)
			if (operator.NOT_EQ_1() != null)
				result = (T) new PyNot1(currentCFG, "", line, col, left, right);

			// Python not equals (!=)
			if (operator.NOT_EQ_2() != null)
				result = (T) new PyNot2(currentCFG, "", line, col, left, right);

			break;
		}

		return result;
	}

	@Override
	public T visitComp_op(Comp_opContext ctx) {

		return super.visitComp_op(ctx);
	}

	@Override
	public T visitStar_expr(Star_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitStar_expr(ctx);
	}

	public T visitExpr(ExprContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nXor = ctx.xor_expr().size();
	
		if (nXor == 1) {
			return visitXor_expr(ctx.xor_expr(0));
		} else if (nXor == 2) {
			return (T) new PyXor(currentCFG, "", line, col, (Expression) visitXor_expr(ctx.xor_expr(0)),
					(Expression) visitXor_expr(ctx.xor_expr(1)));
		} else {
			T temp = (T) new PyXor(currentCFG, "", line, col, (Expression) visitXor_expr(ctx.xor_expr(nXor - 2)),
					(Expression) visitXor_expr(ctx.xor_expr(nXor - 1)));
			nXor = nXor - 2;
			while (nXor > 0) {
				temp = (T) new PyXor(currentCFG, "", line, col, (Expression) visitXor_expr(ctx.xor_expr(--nXor)),
						(Expression) temp);
			}
			return temp;
		}
		
	}

	@Override
	public T visitXor_expr(Xor_exprContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nAnd = ctx.and_expr().size();
	
		if (nAnd == 1) {
			return visitAnd_expr(ctx.and_expr(0));
		} else if (nAnd == 2) {
			return (T) new PyXor(currentCFG, "", line, col, (Expression) visitAnd_expr(ctx.and_expr(0)),
					(Expression) visitAnd_expr(ctx.and_expr(1)));
		} else {
			T temp = (T) new PyXor(currentCFG, "", line, col, (Expression) visitAnd_expr(ctx.and_expr(nAnd - 2)),
					(Expression) visitAnd_expr(ctx.and_expr(nAnd - 1)));
			nAnd = nAnd - 2;
			while (nAnd > 0) {
				temp = (T) new PyXor(currentCFG, "", line, col, (Expression) visitAnd_expr(ctx.and_expr(--nAnd)),
						(Expression) temp);
			}
			return temp;
		}
	}

	@Override
	public T visitAnd_expr(And_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAnd_expr(ctx);
	}

	@Override
	public T visitRight_shift(Right_shiftContext ctx) {
		
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyShiftRight(currentCFG, "", line, col, (Expression)visitArith_expr(ctx.arith_expr()), (Expression)visitShift_expr(ctx.shift_expr()));	
	}
	
	@Override
	public T visitLeft_shift(Left_shiftContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyShiftLeft(currentCFG, "", line, col, (Expression)visitArith_expr(ctx.arith_expr()), (Expression)visitShift_expr(ctx.shift_expr()));	
	}
	
	@Override
	public T visitShift_expr(Shift_exprContext ctx) {
		
		if(ctx.left_shift()!=null) {
			return visitLeft_shift(ctx.left_shift());
		}else if(ctx.right_shift()!=null){
			return visitRight_shift(ctx.right_shift());
		}	
		return null;
	}
	
	@Override
	public T visitMinus(MinusContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyMinus(currentCFG, "", line, col, (Expression)visitTerm(ctx.term()), (Expression)visitArith_expr(ctx.arith_expr()));
	}
	
	@Override
	public T visitAdd(AddContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyAdd(currentCFG, "", line, col, (Expression)visitTerm(ctx.term()), (Expression)visitArith_expr(ctx.arith_expr()));
	}

	@Override
	public T visitArith_expr(Arith_exprContext ctx) {
		if(ctx.minus()!=null) {
			return visitMinus(ctx.minus());
		}
		else{
			return visitAdd(ctx.add());
		}
	}
	
	@Override
	public T visitMul(MulContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyMul(currentCFG, "", line, col, (Expression)visitFactor(ctx.factor()), (Expression)visitTerm(ctx.term()));
	}
	
	public T visitMat_mul(Mat_mulContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyMatMul(currentCFG, "", line, col, (Expression)visitFactor(ctx.factor()), (Expression)visitTerm(ctx.term()));
	}
	
	public T visitDiv(DivContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyDiv(currentCFG, "", line, col, (Expression)visitFactor(ctx.factor()), (Expression)visitTerm(ctx.term()));
	}
	
	public T visitMod(ModContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyMod(currentCFG, "", line, col, (Expression)visitFactor(ctx.factor()), (Expression)visitTerm(ctx.term()));
	}	
	
	public T visitFloorDiv(FloorDivContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		return (T) new PyFloorDiv(currentCFG, "", line, col, (Expression)visitFactor(ctx.factor()), (Expression)visitTerm(ctx.term()));
	}
	@Override
	public T visitTerm(TermContext ctx) {
		if(ctx.mul()!=null) {
			return visitMul(ctx.mul());
		}
		else if(ctx.mat_mul()!=null) {
			return visitMat_mul(ctx.mat_mul());
		}
		else if(ctx.div()!=null) {
			return visitDiv(ctx.div());
		}
		else if(ctx.mod()!=null) {
			return visitMod(ctx.mod());
		}	
		else if(ctx.floorDiv()!=null) {
			return visitFloorDiv(ctx.floorDiv());
		}
		return null;
	}

	@Override
	public T visitFactor(FactorContext ctx) {
		// TODO Auto-generated method stub
		return super.visitFactor(ctx);
	}

	@Override
	public T visitPower(PowerContext ctx) {
		// TODO Auto-generated method stub
		return super.visitPower(ctx);
	}

	@Override
	public T visitAtom_expr(Atom_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAtom_expr(ctx);
	}

	@Override
	public T visitAtom(AtomContext ctx) {
		// TODO Auto-generated method stub
		if (ctx.NAME() != null) {
			// new variabile
			log.info("NAME:");
			log.info(ctx.NAME());
			return (T) new Variable(currentCFG, ctx.NAME().getText());
		} else if (ctx.NUMBER() != null) {
			// literal
			log.info("NUMBER:");
			log.info(ctx.NUMBER());
			return (T) new Literal(currentCFG, ctx.NUMBER().getText(), PyIntType.INSTANCE);
		} else if (ctx.STRING().size() > 0) {
			log.info(ctx.STRING(0));
			//TODO da sisttemare
			return (T) new Literal(currentCFG, ctx.STRING(0).getText(), PyStringType.INSTANCE);
		}
		return super.visitAtom(ctx);
	}

	@Override
	public T visitTestlist_comp(Testlist_compContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTestlist_comp(ctx);
	}

	@Override
	public T visitTrailer(TrailerContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTrailer(ctx);
	}

	@Override
	public T visitSubscriptlist(SubscriptlistContext ctx) {
		// TODO Auto-generated method stub
		return super.visitSubscriptlist(ctx);
	}

	@Override
	public T visitSubscript(SubscriptContext ctx) {
		// TODO Auto-generated method stub
		return super.visitSubscript(ctx);
	}

	@Override
	public T visitSliceop(SliceopContext ctx) {
		// TODO Auto-generated method stub
		return super.visitSliceop(ctx);
	}

	@Override
	public T visitExprlist(ExprlistContext ctx) {
		int expSize=ctx.expr().size();
		int starExpSize=ctx.star_expr().size();
		if(ctx.expr()!=null && ctx.star_expr()==null) {
			
		}
		return super.visitExprlist(ctx);
	}

	@Override
	public T visitTestlist(TestlistContext ctx) {
		Pair<Statement,Statement> firstTest=(Pair<Statement, Statement>) visitTest(ctx.test(0));
		Pair<Statement,Statement> prevTest=firstTest;
		for(int i=1;i<ctx.test().size();i++) {
			Pair<Statement,Statement> currentTest=(Pair<Statement, Statement>) visitTest(ctx.test(i));
			currentCFG.addEdge(new SequentialEdge(prevTest.getRight(), currentTest.getLeft()));
			prevTest=currentTest;
		}
		return (T) Pair.of(firstTest.getLeft(), prevTest.getRight());
	}

	@Override
	public T visitDictorsetmaker(DictorsetmakerContext ctx) {
		// TODO Auto-generated method stub
		return super.visitDictorsetmaker(ctx);
	}

	@Override
	public T visitClassdef(ClassdefContext ctx) {
		// TODO Auto-generated method stub
		return super.visitClassdef(ctx);
	}

	@Override
	public T visitArglist(ArglistContext ctx) {
		// TODO Auto-generated method stub
		return super.visitArglist(ctx);
	}

	@Override
	public T visitArgument(ArgumentContext ctx) {
		// TODO Auto-generated method stub
		return super.visitArgument(ctx);
	}

	@Override
	public T visitComp_iter(Comp_iterContext ctx) {
		// TODO Auto-generated method stub
		return super.visitComp_iter(ctx);
	}

	@Override
	public T visitComp_for(Comp_forContext ctx) {
		// TODO Auto-generated method stub
		return super.visitComp_for(ctx);
	}

	@Override
	public T visitComp_if(Comp_ifContext ctx) {
		// TODO Auto-generated method stub
		return super.visitComp_if(ctx);
	}

	@Override
	public T visitEncoding_decl(Encoding_declContext ctx) {
		// TODO Auto-generated method stub
		return super.visitEncoding_decl(ctx);
	}

	@Override
	public T visitYield_expr(Yield_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitYield_expr(ctx);
	}

	@Override
	public T visitYield_arg(Yield_argContext ctx) {
		// TODO Auto-generated method stub
		return super.visitYield_arg(ctx);
	}

}