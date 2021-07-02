package it.unive.pylisa;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.LiSA;
import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.CFGDescriptor;
import it.unive.lisa.cfg.Parameter;
import it.unive.lisa.cfg.edge.FalseEdge;
import it.unive.lisa.cfg.edge.SequentialEdge;
import it.unive.lisa.cfg.edge.TrueEdge;
import it.unive.lisa.cfg.statement.*;
import it.unive.lisa.logging.IterationLogger;
import it.unive.pylisa.antlr.Python3BaseVisitor;
import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.*;
import it.unive.pylisa.cfg.expression.binary.*;
import it.unive.pylisa.cfg.expression.unary.PyNot;
import it.unive.pylisa.cfg.type.PyFalseLiteral;
import it.unive.pylisa.cfg.type.PyIntType;
import it.unive.pylisa.cfg.type.PyStringLiteral;
import it.unive.pylisa.cfg.type.PyTrueLiteral;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.tree.ParseTree;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.function.Function;

public class PyToCFG extends Python3BaseVisitor<Pair<Statement, Statement>> {

	private static final Logger log = LogManager.getLogger(PyToCFG.class);

	/**
	 * Python program file path.
	 */
	private final String filePath;

	/**
	 * Collection of CFGs collected into the Python program at filePath.
	 */
	private final Collection<CFG> cfgs;

	/**
	 * Builds an instance of @PyToCFG for a given Python program given at the
	 * location filePath.
	 *
	 * @param filePath file path to a Python program.
	 */
	public PyToCFG(String filePath) {
		this.cfgs = new HashSet<>();
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
		//path of test file
		String file = args[0];
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
		currentCFG = new CFG(buildMainCFGDescriptor());
		cfgs.add(currentCFG);
		Statement last_stmt = null;
		for (StmtContext stmt : IterationLogger.iterate(log, ctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Compound_stmtContext comp = stmt.compound_stmt();
			if(comp!=null) {
				visitCompound_stmt(comp);
			}
			else {
				Simple_stmtContext single_stmt = stmt.simple_stmt();
				Pair<Statement, Statement> visited_stmt = visitSimple_stmt(single_stmt);
				currentCFG.addNode(visited_stmt.getLeft(), last_stmt==null);
				if(last_stmt!=null) {
					currentCFG.addEdge(new SequentialEdge(last_stmt, visited_stmt.getLeft()));
				}
				last_stmt = visited_stmt.getRight();
			}
			
		}

		return null;

	}

	private int getLine(ParserRuleContext ctx) {
		return ctx.getStart().getLine();
	}

	private int getCol(ParserRuleContext ctx) {
		return ctx.getStop().getCharPositionInLine();
	}

	private CFGDescriptor buildMainCFGDescriptor() {
		String funcName = "main";
		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(funcName, cfgArgs);
	}

	private CFGDescriptor buildCFGDescriptor(FuncdefContext funcDecl) {
		String funcName = funcDecl.NAME().getText();

		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(funcName, cfgArgs);
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
		throw new UnsupportedStatementException();
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
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitSimple_stmt(Simple_stmtContext ctx) {
			return visitListOfSmallStatements(ctx.small_stmt());
	}

	@Override
	public Pair<Statement, Statement> visitSmall_stmt(Small_stmtContext ctx) {
		return visitChildren(ctx);
	}

	@Override
	public Pair<Statement, Statement> visitExpr_stmt(Expr_stmtContext ctx) {

		Statement assegnazione;
		
		// � un assegnazione
		if (ctx.ASSIGN().size() > 0) {

			Statement target = checkAndExtractSingleStatement(visitChildren(ctx.testlist_star_expr(0)));
			Statement expression = checkAndExtractSingleStatement(visitChildren(ctx.testlist_star_expr(1)));
			log.info(target);
			log.info(expression);
			if((! (target instanceof Expression)) || (! (expression instanceof Expression)))
				throw new UnsupportedStatementException("Assignments require expression both in the left and in the right hand side");
			assegnazione = new Assignment(currentCFG,  (Expression) target,  (Expression) expression);
			log.info(assegnazione);
			currentCFG.addNode( assegnazione);
		}
		else
			return visitTestlistStarExpr(ctx);
		log.info(assegnazione);
		return Pair.of(assegnazione, assegnazione);
	}

	@Override
	public Pair<Statement, Statement> visitAnnassign(AnnassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitTestlist_star_expr(Testlist_star_exprContext ctx) {
		Statement first = null;
		Statement last = null;
		for(TestContext test : ctx.test()) {
			Pair<Statement, Statement> parsed = visitTest(test);
			if(first==null)
				first=parsed.getLeft();
			last = parsed.getRight();
		}
		return Pair.of(first, last);
	}

	@Override
	public Pair<Statement, Statement> visitAugassign(AugassignContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDel_stmt(Del_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitPass_stmt(Pass_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitFlow_stmt(Flow_stmtContext ctx) {
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
		throw new UnsupportedStatementException();
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
		if(ctx.import_from()!=null) {
			return visitImport_from(ctx.import_from());
		}else{
			return visitImport_name(ctx.import_name());
		}
	}

	@Override
	public Pair<Statement, Statement> visitImport_name(Import_nameContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitImport_from(Import_fromContext ctx) {
		throw new UnsupportedStatementException();
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
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitCompound_stmt(Compound_stmtContext ctx) {
		if(ctx.funcdef()!=null) {
			FuncdefContext funcDecl = ctx.funcdef();
			CFG oldCFG = currentCFG;
			currentCFG = new CFG(buildCFGDescriptor(funcDecl));
			cfgs.add(currentCFG);
			visitFuncdef(funcDecl);
			try {
				Writer w = new FileWriter("./output.txt");
				currentCFG.dump(w, "Try");
				w.close();
			} catch (IOException e) {
				log.info("error with file");
				e.printStackTrace();
			}
			currentCFG = oldCFG;
			return null;
		}
		else if(ctx.if_stmt()!=null)
			return this.visitIf_stmt(ctx.if_stmt());
		else if(ctx.while_stmt()!=null)
			return this.visitWhile_stmt(ctx.while_stmt());
		else if(ctx.for_stmt()!=null)
			return this.visitFor_stmt(ctx.for_stmt());
		else if(ctx.try_stmt()!=null)
			return this.visitTry_stmt(ctx.try_stmt());
		else if(ctx.with_stmt()!=null)
			return this.visitWith_stmt(ctx.with_stmt());
		else if(ctx.if_stmt()!=null)
			return this.visitIf_stmt(ctx.if_stmt());
		else throw new UnsupportedStatementException("Statement "+ctx+" not yet supported");
	}

	@Override
	public Pair<Statement, Statement> visitAsync_stmt(Async_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	private Statement checkAndExtractSingleStatement(Pair<Statement, Statement> parsed) {
		if(parsed.getLeft()!=parsed.getRight())
			throw new UnsupportedStatementException("It is not supported to have multiple expressions here");
		return parsed.getLeft();
	}

	private Expression checkAndExtractSingleExpression(Pair<Statement, Statement> parsed) {
		Statement result = checkAndExtractSingleStatement(parsed);
		if(! (result instanceof Expression))
			throw new UnsupportedStatementException("An expression is expected here");
		return (Expression) result;
	}

	private <T> Pair<T, T> createPairFromSingle(T st) {
		return Pair.of(st, st);
	}

	@Override
	public Pair<Statement, Statement> visitIf_stmt(If_stmtContext ctx) {

		Statement booleanGuard =  checkAndExtractSingleStatement(visitTest(ctx.test(0)));
		currentCFG.addNode(booleanGuard);

		//Created if exit node
		NoOp ifExitNode = new NoOp(currentCFG);
		currentCFG.addNode(ifExitNode);

		//Visit if true block
		Pair<Statement, Statement> trueBlock = visitSuite(ctx.suite(0));


		Statement exitStatementTrueBranch = trueBlock.getRight();
		Statement entryStatementTrueBranch = trueBlock.getLeft();

		currentCFG.addNode(exitStatementTrueBranch);
		currentCFG.addNode(entryStatementTrueBranch);
		
		currentCFG.addEdge(new TrueEdge(booleanGuard, entryStatementTrueBranch));
		currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranch, ifExitNode));
	
		int testLenght = ctx.test().size();

		Statement lastBooleanGuardElif = booleanGuard;
		
		// if testLenght is >1 the context contains elif	
		if(testLenght>1) {

				int i = 1;
				
				// visit all the elif
				while (i < testLenght) {
					Statement booleanGuardElif = checkAndExtractSingleStatement(visitTest(ctx.test(i)));
					currentCFG.addNode(booleanGuardElif);
					currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, booleanGuardElif));
					lastBooleanGuardElif = booleanGuardElif;
					Pair<Statement, Statement> trueBlockElif = visitSuite(ctx.suite(i));

					Statement exitStatementTrueBranchElif = trueBlockElif.getRight();
					Statement entryStatementTrueBranchElif = trueBlockElif.getLeft();
					
					currentCFG.addNode(exitStatementTrueBranchElif);
					currentCFG.addNode(entryStatementTrueBranchElif);
					
					currentCFG.addEdge(new TrueEdge(booleanGuardElif, entryStatementTrueBranchElif));
					currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranchElif, ifExitNode));
					i=i+1;
				}
		}
				// If statement with else
				if (ctx.ELSE() !=null) {
		
					Pair<Statement, Statement> falseBlock = visitSuite(ctx.suite(ctx.suite().size()-1));
					Statement entryStatementFalseBranch = falseBlock.getLeft();
					Statement exitStatementFalseBranch = falseBlock.getRight();
					currentCFG.addNode(entryStatementFalseBranch);
					currentCFG.addNode(exitStatementFalseBranch);
					
					currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, entryStatementFalseBranch));
					currentCFG.addEdge(new SequentialEdge(exitStatementFalseBranch, ifExitNode));
				}else {
					// If statement with no else
					currentCFG.addEdge(new FalseEdge(lastBooleanGuardElif, ifExitNode));
				}

		return  Pair.of(booleanGuard,ifExitNode);
	}

	@Override
	public Pair<Statement, Statement> visitWhile_stmt(While_stmtContext ctx) {
		
		//create and add exit point of while
		NoOp whileExitNode = new NoOp(currentCFG);
		currentCFG.addNode(whileExitNode);

		Statement condition  = checkAndExtractSingleStatement(visitTest(ctx.test()));
		currentCFG.addNode(condition);

		Pair<Statement, Statement> trueBlock = visitSuite(ctx.suite(0));
		
		currentCFG.addEdge(new TrueEdge(condition, trueBlock.getLeft()));
		currentCFG.addEdge(new SequentialEdge(trueBlock.getRight(), condition));
		
		//check if there's an else condition for the while
		if(ctx.ELSE()!=null) {
			Pair<Statement, Statement> falseBlock = visitSuite(ctx.suite(1));
			currentCFG.addEdge(new FalseEdge(condition, falseBlock.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseBlock.getRight(), whileExitNode));
		}else {
			currentCFG.addEdge(new FalseEdge(condition, whileExitNode));
		}
		
		
		return  Pair.of(condition, whileExitNode);
	}

	@Override
	public Pair<Statement, Statement> visitFor_stmt(For_stmtContext ctx) {
		
		//create and add exit point of for
		NoOp forExitNode = new NoOp(currentCFG);
		currentCFG.addNode(forExitNode);
		
		Pair<Statement, Statement> exprlist= visitExprlist(ctx.exprlist());
		
		Pair<Statement, Statement> testList= visitTestlist(ctx.testlist());
		
		currentCFG.addEdge(new SequentialEdge(exprlist.getRight(), testList.getLeft()));
		
		Pair<Statement, Statement> body= visitSuite(ctx.suite(0));
		
		currentCFG.addEdge(new TrueEdge(testList.getRight(), body.getLeft()));
		currentCFG.addEdge(new TrueEdge(exprlist.getLeft(), body.getRight()));
		
		//check if there's an else condition for the for statement
		if(ctx.ELSE()!=null) {
			Pair<Statement, Statement> falseCond= visitSuite(ctx.suite(1));
			currentCFG.addEdge(new FalseEdge(testList.getLeft(), falseCond.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseCond.getRight(), forExitNode));
		}
		return Pair.of(exprlist.getLeft(), forExitNode);
	}

	@Override
	public Pair<Statement, Statement> visitTry_stmt(Try_stmtContext ctx) {
		//this method is commented because the type of edge required does not exist now
		
		/*
		
		NoOp TryExitNode = new NoOp(currentCFG);
		currentCFG.addNode(TryExitNode);
		
		Pair<Statement, Statement> tryBlock = (Pair<Statement, Statement>) visitSuite(ctx.suite(0));
		
		int nException=ctx.except_clause().size();
		
		Pair<Statement, Statement> firstClauseException = (Pair<Statement, Statement>) visitExcept_clause(ctx.except_clause(0));
		
		Pair<Statement, Statement> firstSuiteException = (Pair<Statement, Statement>) visitSuite(ctx.suite(1));
		
		currentCFG.addEdge(new TrueEdge(tryBlock.getRight(), firstClauseException.getLeft()));
		
		currentCFG.addEdge(new SequentialEdge(firstClauseException.getRight(), firstSuiteException.getLeft()));
		
		Pair<Statement, Statement> lastClauseExc=firstClauseException;
		Pair<Statement, Statement> lastSuiteExc=firstSuiteException;
		
		for(int i=2;i<=nException;i++) {
			
			Pair<Statement, Statement> currentClauseExc = (Pair<Statement, Statement>) visitSuite(ctx.suite(i));
			Pair<Statement, Statement>  currentSuiteExc= (Pair<Statement, Statement>)  visitExcept_clause(ctx.except_clause(i-1));
			
			currentCFG.addEdge(new SequentialEdge(currentClauseExc.getRight(), currentSuiteExc.getLeft()));
			currentCFG.addEdge(new SequentialEdge(lastSuiteExc.getRight(), currentClauseExc.getLeft()));
			
			lastSuiteExc=currentSuiteExc;
			lastClauseExc=currentClauseExc;
		}
		
		if(ctx.ELSE()!=null) {
			int posElseSuite=1+nException;
			Pair<Statement, Statement> elseBlock = (Pair<Statement, Statement>) visitSuite(ctx.suite(posElseSuite));
			
			currentCFG.addEdge(new SequentialEdge(lastSuiteExc.getRight(), currentClauseExc.getLeft()));
		}
		
		*/
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitWith_stmt(With_stmtContext ctx) {
		int withSize=ctx.with_item().size();
		Pair<Statement, Statement> withItem=visitWith_item(ctx.with_item(0));
		Pair<Statement, Statement> prev=withItem;
		Pair<Statement, Statement> curr=withItem;
		for(int i=1;i<withSize;i++) {
			curr= visitWith_item(ctx.with_item(i));
			currentCFG.addEdge(new SequentialEdge(prev.getRight(), curr.getLeft()));
			prev=curr;
		}
		Pair<Statement, Statement> suite= visitSuite(ctx.suite());
		currentCFG.addNode(suite.getLeft());
		currentCFG.addNode(suite.getRight());
		currentCFG.addEdge(new SequentialEdge(curr.getRight(),suite.getLeft()));
		
		return  Pair.of(withItem.getLeft(),suite.getRight());
	}

	@Override
	public Pair<Statement, Statement> visitWith_item(With_itemContext ctx) {
		Statement test= checkAndExtractSingleStatement(visitTest(ctx.test()));
		currentCFG.addNode(test);
		if(ctx.expr()!=null) {
			Statement expr = checkAndExtractSingleStatement(visitExpr(ctx.expr()));
			currentCFG.addNode(expr);
			currentCFG.addEdge(new SequentialEdge(test, expr));
			return Pair.of(test,expr);
		}else {
			return Pair.of(test,test);
		}
		
	}

	@Override
	public Pair<Statement, Statement> visitExcept_clause(Except_clauseContext ctx) {
		throw new UnsupportedStatementException();
	}

	private <T> Pair<Statement, Statement> visitListOf(List<T> elements, Function<T, Pair<Statement, Statement>> visitor) {
		Statement result = null;
		Statement first = null;
		for(T element : elements) {
			Statement previous = result;
			Pair<Statement, Statement> parsed = visitor.apply(element);
			if(first==null)
				first=result;
			if(previous!=null)
				currentCFG.addEdge(new SequentialEdge(previous, parsed.getLeft()));
			result = parsed.getRight();
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
		//for(Testlist_star_exprContext tstlist : ctx.testlist_star_expr())
		//	visitTestlist_star_expr(tstlist);
		return visitListOf(ctx.testlist_star_expr(), this::visitTestlist_star_expr);
	}


	@Override
	public Pair<Statement, Statement> visitTestlist(TestlistContext ctx) {
		return visitListOf(ctx.test(), this::visitTest);
	}
	@Override
	public Pair<Statement, Statement> visitExprlist(ExprlistContext ctx) {
		if(ctx.expr().size()>=1 && ctx.star_expr().size()>=1) {
			Pair<Statement,Statement> expPair= visitListOf(ctx.expr(), this::visitExpr);
			Pair<Statement,Statement> starExpPair = visitListOf(ctx.star_expr(), this::visitStar_expr);
			return Pair.of(expPair.getLeft(), starExpPair.getRight());
		}
		else if(ctx.expr().size()>=1)
			return visitListOf(ctx.expr(), this::visitExpr);
		else if(ctx.star_expr().size()>=1)
			return visitListOf(ctx.star_expr(), this::visitStar_expr);
		else throw new UnsupportedStatementException("We need to have at least expressions or start expressions");
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
		//no if into the condition 
		if (ctx.IF() == null) {
			return visitOr_test(ctx.or_test(0));
		} else {
			//visit the if into the condition
			Statement trueCase = checkAndExtractSingleStatement(visitOr_test(ctx.or_test(0)));
			Statement booleanGuard = checkAndExtractSingleStatement(visitOr_test(ctx.or_test(1)));
			Pair<Statement,Statement> falseCase = visitTest(ctx.test());
			NoOp testExitNode = new NoOp(currentCFG);
			
			currentCFG.addEdge(new TrueEdge(booleanGuard, trueCase));
			currentCFG.addEdge(new FalseEdge(booleanGuard, falseCase.getLeft()));
			currentCFG.addEdge(new SequentialEdge(falseCase.getRight(), testExitNode));
			currentCFG.addEdge(new SequentialEdge(trueCase, testExitNode));

			return Pair.of(booleanGuard, testExitNode);
		}

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
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nAndTest = ctx.and_test().size();
		if (nAndTest == 1) {
			return visitAnd_test(ctx.and_test(0));
		} else if (nAndTest == 2) {
			return createPairFromSingle(new PyOr(currentCFG, "", line, col, checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(0))),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(1))),"or"));
		} else {
			Expression temp = new PyOr(currentCFG, "", line, col, checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(nAndTest - 2))),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(nAndTest - 1))),"or");
			nAndTest = nAndTest - 2;
			while (nAndTest > 0) {
				temp = new PyOr(currentCFG, "", line, col, checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(--nAndTest))),
						temp,"or");
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitAnd_test(And_testContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nNotTest = ctx.not_test().size();
		if (nNotTest == 1) {
			return visitNot_test(ctx.not_test(0));
		} else if (nNotTest == 2) {
			return createPairFromSingle(new PyAnd(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitNot_test(ctx.not_test(0))),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(1)))));
		} else {
			Expression temp =  new PyAnd(currentCFG, "", line, col, checkAndExtractSingleExpression(visitNot_test(ctx.not_test(nNotTest - 2))),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(nNotTest - 1))));
			nNotTest = nNotTest - 2;
			while (nNotTest > 0) {
				temp = new PyAnd(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitNot_test(ctx.not_test(--nNotTest))),
						 temp);
			}

			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitNot_test(Not_testContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);

		if (ctx.NOT() != null) {
			return createPairFromSingle(new PyNot(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitNot_test(ctx.not_test()))));
		} else {
			return visitComparison(ctx.comparison());
		}

	}

	@Override
	public Pair<Statement, Statement> visitComparison(ComparisonContext ctx) {

		int line = getLine(ctx);
		int col = getCol(ctx);

		int nExpr = ctx.expr().size();
		Expression result = null;
		switch (nExpr) {
		case 1:
			result = checkAndExtractSingleExpression(visitExpr(ctx.expr(0)));
			break;
		case 2:

			Comp_opContext operator = ctx.comp_op(0);
			Expression left =  checkAndExtractSingleExpression(visitExpr(ctx.expr(0)));
			Expression right =  checkAndExtractSingleExpression(visitExpr(ctx.expr(1)));
			if (operator.EQUALS() != null)
				result = new PyEquals(currentCFG, "", line, col, left, right);

			// Python greater (>)
			if (operator.GREATER_THAN() != null) {
				result = new PyGreater(currentCFG, "", line, col, left, right);
			}
			// Python greater equal (>=)
			if (operator.GT_EQ() != null)
				result = new PyEquals(currentCFG, "", line, col, left, right);

			// Python in (in)
			if (operator.IN() != null)
				result = new PyIn(currentCFG, "", line, col, left, right);

			// Python is (is)
			if (operator.IS() != null)
				result = new PyIs(currentCFG, "", line, col, left, right);

			// Python less (<)
			if (operator.LESS_THAN() != null)
				result = new PyLess(currentCFG, "", line, col, left, right);

			// Python less equal (<=)
			if (operator.LT_EQ() != null)
				result = new PyLessEqual(currentCFG, "", line, col, left, right);

			// Python not (not)
			if (operator.NOT() != null)
				result =  new PyNot(currentCFG, "", line, col, left);

			// Python not equals (<>)
			if (operator.NOT_EQ_1() != null)
				result = new PyNot1(currentCFG, "", line, col, left, right);

			// Python not equals (!=)
			if (operator.NOT_EQ_2() != null)
				result = new PyNot2(currentCFG, "", line, col, left, right);

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
		int line = getLine(ctx);
		int col = getCol(ctx);

		int nXor = ctx.xor_expr().size();
	
		if (nXor == 1) {
			//only one Xor
			return visitXor_expr(ctx.xor_expr(0));
		} else if (nXor == 2) {
			//two Xor
			return createPairFromSingle(new PyXor(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(0))),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(1)))));
		} else {
			Expression temp = new PyXor(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(nXor - 2))),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(nXor - 1))));
			nXor = nXor - 2;
			// concatenate all the Xor expressions together
			while (nXor > 0) {
				temp = new PyXor(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(--nXor))),
						 temp);
			}
			return createPairFromSingle(temp);
		}
		
	}

	@Override
	public Pair<Statement, Statement> visitXor_expr(Xor_exprContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
	
		int nAnd = ctx.and_expr().size();
	
		if (nAnd == 1) {
			return visitAnd_expr(ctx.and_expr(0));
		} else if (nAnd == 2) {
			return createPairFromSingle(new PyXor(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(0))),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(1)))));
		} else {
			Expression temp = new PyXor(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(nAnd - 2))),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(nAnd - 1))));
			nAnd = nAnd - 2;
			// concatenate all the And expressions together
			while (nAnd > 0) {
				temp = new PyXor(currentCFG, "", line, col, checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(--nAnd))),
						 temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitAnd_expr(And_exprContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		int nShift = ctx.shift_expr().size();
	
		if (nShift == 1) {
			return visitShift_expr(ctx.shift_expr(0));
		} else if (nShift == 2) {
			return createPairFromSingle(new PyAnd(currentCFG, "", line, col,  checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(0))),
					checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(1)))));
		} else {
			Expression temp = new PyAnd(currentCFG, "", line, col, checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(nShift - 2))),
					checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(nShift - 1))));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyAnd(currentCFG, "", line, col, checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(--nShift))),
						 temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitRight_shift(Right_shiftContext ctx) {

		int line = getLine(ctx);
		int col = getCol(ctx);
	
		if (ctx.RIGHT_SHIFT()==null) {
			return visitArith_expr(ctx.arith_expr());
		} else  {
			return createPairFromSingle(new PyShiftRight(currentCFG, "", line, col, checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr())), checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr()))));
		}
	}
	
	@Override
	public Pair<Statement, Statement> visitLeft_shift(Left_shiftContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.LEFT_SHIFT()==null) {
			return visitArith_expr(ctx.arith_expr());
		} else  {
			return createPairFromSingle(new PyShiftLeft(currentCFG, "", line, col, checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr())), checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr()))));
		}	
	}
	
	@Override
	public Pair<Statement, Statement> visitShift_expr(Shift_exprContext ctx) {
		
		//check if there is a right shift expression(>>) or a left shift expression(<<)
		if(ctx.left_shift()!=null) {
			return visitLeft_shift(ctx.left_shift());
		}else if(ctx.right_shift()!=null){
			return visitRight_shift(ctx.right_shift());
		}
		throw new UnsupportedStatementException();
	}
	
	@Override
	public Pair<Statement, Statement> visitMinus(MinusContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.arith_expr()==null) {
			return visitTerm(ctx.term());
		} else  {
			return createPairFromSingle(new PyMinus(currentCFG, "", line, col, checkAndExtractSingleExpression(visitTerm(ctx.term())), checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr()))));
		}
	}
	
	@Override
	public Pair<Statement, Statement> visitAdd(AddContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.arith_expr()==null) {
			return visitTerm(ctx.term());
		} else  {
			return createPairFromSingle(new PyAdd(currentCFG, "", line, col, checkAndExtractSingleExpression(visitTerm(ctx.term())), checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr()))));
		}
		
	}

	@Override
	public Pair<Statement, Statement> visitArith_expr(Arith_exprContext ctx) {
		
		//check if there is minus(-) or an add(+)
		if(ctx.minus()!=null) {
			return visitMinus(ctx.minus());
		}
		else{
			return visitAdd(ctx.add());
		}
	}
	
	@Override
	public Pair<Statement, Statement> visitMul(MulContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyMul(currentCFG, "", line, col, checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}	
		
	}
	
	public Pair<Statement, Statement> visitMat_mul(Mat_mulContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyMatMul(currentCFG, "", line, col, checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}	
		
		
	}
	
	public Pair<Statement, Statement> visitDiv(DivContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyDiv(currentCFG, "", line, col, checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}	

		
	}
	
	public Pair<Statement, Statement> visitMod(ModContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyMod(currentCFG, "", line, col, checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}
		
	}	
	
	public Pair<Statement, Statement> visitFloorDiv(FloorDivContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyFloorDiv(currentCFG, "", line, col, checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}		
	}
	
	@Override
	public Pair<Statement, Statement> visitTerm(TermContext ctx) {
		//check what's the operation in the context
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
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitFactor(FactorContext ctx) {
		if(ctx.power()!=null) {
			return visitPower(ctx.power());
		}
		return super.visitFactor(ctx);
	}

	@Override
	public Pair<Statement, Statement> visitPower(PowerContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if(ctx.POWER()!=null) {
			return createPairFromSingle(new PyPower(currentCFG, "", line, col, checkAndExtractSingleExpression(visitAtom_expr(ctx.atom_expr())), checkAndExtractSingleExpression(visitFactor(ctx.factor()))));
		}
		else throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitAtom_expr(Atom_exprContext ctx) {
		return visitAtom(ctx.atom());
	}

	@Override
	public Pair<Statement, Statement> visitAtom(AtomContext ctx) {
		int line = getLine(ctx);
		int col = getCol(ctx);
		
		if (ctx.NAME() != null) {
			//crete a variable
			return createPairFromSingle(new Variable(currentCFG, ctx.NAME().getText()));
		} else if (ctx.NUMBER() != null) {
			//create a literal int
			return createPairFromSingle(new Literal(currentCFG, ctx.NUMBER().getText(), PyIntType.INSTANCE));
		}else if (ctx.FALSE() != null) {
			//create a literal false
			return createPairFromSingle(new PyFalseLiteral(currentCFG, "",line,col));
		}else if (ctx.TRUE()!= null) {
			//create a literal true
			return createPairFromSingle(new PyTrueLiteral(currentCFG, "",line,col));
		} else if (ctx.STRING().size() > 0) {
			//create a string
			return createPairFromSingle(new PyStringLiteral(currentCFG,"", line, col, ctx.STRING(0).getText()));
		} else if(ctx.yield_expr()!=null) {
			return visitYield_expr(ctx.yield_expr());
		} else if (ctx.testlist_comp()!=null) {
			return visitTestlist_comp(ctx.testlist_comp());
		} else if(ctx.dictorsetmaker()!=null) {
			return visitDictorsetmaker(ctx.dictorsetmaker());
		}
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitTestlist_comp(Testlist_compContext ctx) {
		Pair<Statement,Statement> elem =visitTestOrStar(ctx.testOrStar(0));
		Pair<Statement,Statement> prev;
		Pair<Statement,Statement> current;
		if(ctx.comp_for()!=null) {			
			Pair<Statement,Statement> forClause= visitComp_for(ctx.comp_for());
			currentCFG.addEdge(new SequentialEdge(elem.getRight(), forClause.getLeft()));
			int nTestOrStar=ctx.testOrStar().size();
			prev=forClause;
			for(int i=1; i<nTestOrStar;i++) {
				current=visitTestOrStar(ctx.testOrStar(i));
				currentCFG.addEdge(new SequentialEdge(prev.getRight(), current.getLeft()));
				prev=current;
			}
			return Pair.of(elem.getLeft(), prev.getRight());
		}
		else
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
	public Pair<Statement, Statement> visitSubscript(SubscriptContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitSliceop(SliceopContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDictorsetmaker(DictorsetmakerContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitClassdef(ClassdefContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitArglist(ArglistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitArgument(ArgumentContext ctx) {
		throw new UnsupportedStatementException();
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

}