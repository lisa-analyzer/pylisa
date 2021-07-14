package it.unive.pylisa;

import it.unive.lisa.AnalysisException;
import it.unive.lisa.LiSA;
import it.unive.lisa.LiSAConfiguration;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.edge.FalseEdge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.edge.TrueEdge;
import it.unive.lisa.program.cfg.statement.*;
import it.unive.lisa.logging.IterationLogger;
import it.unive.pylisa.antlr.Python3BaseVisitor;
import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
import it.unive.pylisa.antlr.Python3Parser.*;
import it.unive.pylisa.cfg.PythonUnit;
import it.unive.pylisa.cfg.expression.binary.*;
import it.unive.pylisa.cfg.expression.unary.PyNot;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.cfg.type.PyFalseLiteral;
import it.unive.pylisa.cfg.type.PyIntType;
import it.unive.pylisa.cfg.type.PyStringLiteral;
import it.unive.pylisa.cfg.type.PyTrueLiteral;
import it.unive.pylisa.cfg.type.PyNoneLiteral;
import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.RuleContext;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.TerminalNode;
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
	 * The unit currently under parsing
	 */
	private Unit currentUnit;

	/**
	 * All the units completely processed so far
	 */
	private final Collection<Unit> parsedUnits = new ArrayList<>();

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
		this.currentUnit = new PythonUnit(new SourceCodeLocation(filePath, 0, 0), "main_file");
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
		LiSAConfiguration conf = new LiSAConfiguration();

		Collection<CFG> cfgs = translator.toLiSACFG();
		Program p = new Program();
		cfgs.forEach(p::addCFG);
		conf.setDumpCFGs(true);
		conf.setWorkdir("workdir");
		LiSA lisa = new LiSA(conf);
		lisa.run(p);
		translator.parsedUnits.add(translator.currentUnit);
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


	public CodeLocation getLocation(ParserRuleContext ctx) {
		return new SourceCodeLocation(this.getFilePath(), getLine(ctx), getCol(ctx));
	}

	private CFGDescriptor buildMainCFGDescriptor() {
		String funcName = "main";
		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(currentUnit, false, funcName, cfgArgs);
	}

	private CFGDescriptor buildCFGDescriptor(FuncdefContext funcDecl) {
		String funcName = funcDecl.NAME().getText();

		Parameter[] cfgArgs = new Parameter[] {};

		return new CFGDescriptor(currentUnit, false, funcName, cfgArgs);
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
		if(ctx.simple_stmt()!=null)
			return visitSimple_stmt(ctx.simple_stmt());
		else return visitCompound_stmt(ctx.compound_stmt());
	}

	@Override
	public Pair<Statement, Statement> visitSimple_stmt(Simple_stmtContext ctx) {
			return visitListOfSmallStatements(ctx.small_stmt());
	}

	@Override
	public Pair<Statement, Statement> visitSmall_stmt(Small_stmtContext ctx) {
		Pair<Statement, Statement> result = visitChildren(ctx);
		//Need to add the nodes since they might have been processed as expressions (thus not yet added to the CFG)
		currentCFG.addNode(result.getLeft());
		currentCFG.addNode(result.getRight());
		return result;
	}

	@Override
	public Pair<Statement, Statement> visitExpr_stmt(Expr_stmtContext ctx) {

		Statement assegnazione;
		
		// è un assegnazione
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
		return createPairFromSingle(assegnazione);
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
		if(ctx.exprlist().star_expr().size()>0)
			throw new UnsupportedStatementException("We support only expressions withou * in del statements");

		return createPairFromSingle(
				new UnresolvedCall(
						currentCFG,
						getLocation(ctx),
						UnresolvedCall.ResolutionStrategy.DYNAMIC_TYPES,
						false,
						"del",
						extractExpressionsFromExprlist(ctx.exprlist()).toArray(new Expression[ctx.exprlist().expr().size()])
				)
		);

	}

	@Override
	public Pair<Statement, Statement> visitPass_stmt(Pass_stmtContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitFlow_stmt(Flow_stmtContext ctx) {
		if(ctx.return_stmt()!=null)
			return visitReturn_stmt(ctx.return_stmt());
		if(ctx.raise_stmt()!=null) {
			log.warn("Exceptions are not yet supported. The raise statement at line "+getLine(ctx)+ " of file "+getFilePath()+" is unsoundly translated into a return; statement");
			return createPairFromSingle(new Ret(currentCFG, getLocation(ctx)));
		}
		if(ctx.yield_stmt()!=null) {
			List<Expression> l = extractExpressionsFromYieldArg(ctx.yield_stmt().yield_expr().yield_arg());
			return createPairFromSingle(
				new UnresolvedCall(
					currentCFG,
					getLocation(ctx),
					UnresolvedCall.ResolutionStrategy.DYNAMIC_TYPES,
					false,
					"yield from",
					l.toArray(new Expression[l.size()])
				)
			);
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
		if(ctx.testlist()==null)
			return createPairFromSingle(new Ret(currentCFG, getLocation(ctx)));
		else {
			if(ctx.testlist().test().size()==1)
				return createPairFromSingle(new Return(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitTestlist(ctx.testlist()))));
			else
				return createPairFromSingle(new TupleCreation(extractExpressionsFromTestlist(ctx.testlist()), currentCFG, getLocation(ctx)));
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
		if(ctx.import_from()!=null) {
			return visitImport_from(ctx.import_from());
		}else{
			return visitImport_name(ctx.import_name());
		}
	}


	/**
	 * This method transforms a single rule to a sequence of statements representing it
	 *
	 * @param ctx the context that internally contains a list of something, and that should be translated into a sequence of statements
	 * @param defaultStatement the default statement to be returned if the list is empty
	 * @param condition if false, the method returns the default statement, otherwise it processes the list
	 * @param getList returns the list of the elements to be processed
	 * @param extractStatement transforms an element into a statement representing it
	 * @param <T1> the context to be processed
	 * @param <T2> the elements to be transformed into statements
	 * @return the first and last statement produced when transforming the context into statements
	 */
	private <T1 extends RuleContext, T2> Pair<Statement, Statement> visitListOfContexts(T1 ctx, Statement defaultStatement, Function<T1, Boolean> condition, Function<T1, List<T2>> getList, Function<T2, Statement> extractStatement) {
		if(condition.apply(ctx)) {
			Statement first = null;
			Statement prev;
			Statement last = null;
			for(T2 single: getList.apply(ctx)) {
				prev=last;
				last = extractStatement.apply(single);
				currentCFG.addNode(last, false);
				if(prev!=null)
					currentCFG.addEdge(new SequentialEdge(prev, last));
				else if(first==null)
					first = last;
			}
			return Pair.of(first, last);
		}
		else
			return createPairFromSingle(defaultStatement);
	}

	@Override
	public Pair<Statement, Statement> visitImport_from(Import_fromContext ctx) {
		String name;
		if(ctx.dotted_name()!=null)
			name = dottedNameToString(ctx.dotted_name());
		else name = ".";
		
		
		Statement def = new FromImport(name, "*", "*", currentCFG, getLocation(ctx));

		return visitListOfContexts(ctx, def,
				c -> c.import_as_names()!=null,
				c -> c.import_as_names().import_as_name(),
				s -> {
					String component = s.NAME(0).getSymbol().getText();
					String asName = s.NAME(1) != null ? s.NAME(1).getSymbol().getText() : component;
					int l = getLine(s);
					int c = getCol(s);
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
					int line = getLine(c);
					int col = getCol(c);
					return new Import(importedLibrary, as, currentCFG, getLocation(c));
				}
		);
	}

	private String dottedNameToString(Dotted_nameContext dotted_name) {
		String result = "";
		boolean first = true;
		for(TerminalNode name : dotted_name.NAME()) {
			if (first)
				first = false;
			else result += ".";
			result += name.getSymbol().getText();
		}
		return result;
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
		return createPairFromSingle(
				new UnresolvedCall(
						currentCFG,
						getLocation(ctx),
						UnresolvedCall.ResolutionStrategy.DYNAMIC_TYPES,
						false,
						"assert",
						extractExpressionsFromListOfTests(ctx.test()).toArray(new Expression[ctx.test().size()])
				)
		);
	}

	private CFG parseMethod(FuncdefContext ctx) {
		CFG oldCFG = currentCFG;
		currentCFG = new CFG(buildCFGDescriptor(ctx));
		cfgs.add(currentCFG);
		visitSuite(ctx.suite());
		try {
			Writer w = new FileWriter("./output.txt");
			currentCFG.dump(w, s -> "Try");
			w.close();
		} catch (IOException e) {
			log.info("error with file");
			e.printStackTrace();
		}
		CFG result = currentCFG;
		currentCFG = oldCFG;
		currentUnit.addCFG(result);
		return result;
	}

	@Override
	public Pair<Statement, Statement> visitCompound_stmt(Compound_stmtContext ctx) {
		if(ctx.funcdef()!=null) {
			parseMethod(ctx.funcdef());
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
		else if(ctx.classdef()!=null)
			return this.visitClassdef(ctx.classdef());
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
		//FIXME: this translation of for loops is simply wrong
		currentCFG.addNode(exprlist.getRight());
		currentCFG.addNode(testList.getLeft());
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
		log.warn("Exceptions are not yet supported. The try block at line "+getLine(ctx)+" of file "+getFilePath()+" is unsoundly translated considering only the code in the try block");
		return visitSuite(ctx.suite(0));
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
			return createPairFromSingle(test);
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
			if(previous!=null)
				currentCFG.addEdge(new SequentialEdge(previous, parsed.getLeft()));
			result = parsed.getRight();
			if(first==null)
				first=result;
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
		
		

		int nAndTest = ctx.and_test().size();
		if (nAndTest == 1) {
			return visitAnd_test(ctx.and_test(0));
		} else if (nAndTest == 2) {
			return createPairFromSingle(new PyOr(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(0))),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(1))),"or"));
		} else {
			Expression temp = new PyOr(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(nAndTest - 2))),
					checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(nAndTest - 1))),"or");
			nAndTest = nAndTest - 2;
			while (nAndTest > 0) {
				temp = new PyOr(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitAnd_test(ctx.and_test(--nAndTest))),
						temp,"or");
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
			return createPairFromSingle(new PyAnd(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitNot_test(ctx.not_test(0))),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(1)))));
		} else {
			Expression temp =  new PyAnd(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitNot_test(ctx.not_test(nNotTest - 2))),
					checkAndExtractSingleExpression(visitNot_test(ctx.not_test(nNotTest - 1))));
			nNotTest = nNotTest - 2;
			while (nNotTest > 0) {
				temp = new PyAnd(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitNot_test(ctx.not_test(--nNotTest))),
						 temp);
			}

			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitNot_test(Not_testContext ctx) {
		
		

		if (ctx.NOT() != null) {
			return createPairFromSingle(new PyNot(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitNot_test(ctx.not_test()))));
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
			Expression left =  checkAndExtractSingleExpression(visitExpr(ctx.expr(0)));
			Expression right =  checkAndExtractSingleExpression(visitExpr(ctx.expr(1)));
			if (operator.EQUALS() != null)
				result = new PyEquals(currentCFG, getLocation(ctx), left, right);

			// Python greater (>)
			if (operator.GREATER_THAN() != null) {
				result = new PyGreater(currentCFG, getLocation(ctx), left, right);
			}
			// Python greater equal (>=)
			if (operator.GT_EQ() != null)
				result = new PyEquals(currentCFG, getLocation(ctx), left, right);

			// Python in (in)
			if (operator.IN() != null)
				result = new PyIn(currentCFG, getLocation(ctx), left, right);

			// Python is (is)
			if (operator.IS() != null)
				result = new PyIs(currentCFG, getLocation(ctx), left, right);

			// Python less (<)
			if (operator.LESS_THAN() != null)
				result = new PyLess(currentCFG, getLocation(ctx), left, right);

			// Python less equal (<=)
			if (operator.LT_EQ() != null)
				result = new PyLessEqual(currentCFG, getLocation(ctx), left, right);

			// Python not (not)
			if (operator.NOT() != null)
				result =  new PyNot(currentCFG, getLocation(ctx), left);

			// Python not equals (<>)
			if (operator.NOT_EQ_1() != null)
				result = new PyNot1(currentCFG, getLocation(ctx), left, right);

			// Python not equals (!=)
			if (operator.NOT_EQ_2() != null)
				result = new PyNot2(currentCFG, getLocation(ctx), left, right);

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
			//only one Xor
			return visitXor_expr(ctx.xor_expr(0));
		} else if (nXor == 2) {
			//two Xor
			return createPairFromSingle(new PyXor(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(0))),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(1)))));
		} else {
			Expression temp = new PyXor(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(nXor - 2))),
					checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(nXor - 1))));
			nXor = nXor - 2;
			// concatenate all the Xor expressions together
			while (nXor > 0) {
				temp = new PyXor(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitXor_expr(ctx.xor_expr(--nXor))),
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
			return createPairFromSingle(new PyXor(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(0))),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(1)))));
		} else {
			Expression temp = new PyXor(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(nAnd - 2))),
					checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(nAnd - 1))));
			nAnd = nAnd - 2;
			// concatenate all the And expressions together
			while (nAnd > 0) {
				temp = new PyXor(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitAnd_expr(ctx.and_expr(--nAnd))),
						 temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitAnd_expr(And_exprContext ctx) {
		
		
		int nShift = ctx.shift_expr().size();
	
		if (nShift == 1) {
			return visitShift_expr(ctx.shift_expr(0));
		} else if (nShift == 2) {
			return createPairFromSingle(new PyAnd(currentCFG, getLocation(ctx),  checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(0))),
					checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(1)))));
		} else {
			Expression temp = new PyAnd(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(nShift - 2))),
					checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(nShift - 1))));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyAnd(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr(--nShift))),
						 temp);
			}
			return createPairFromSingle(temp);
		}
	}

	@Override
	public Pair<Statement, Statement> visitRight_shift(Right_shiftContext ctx) {

		
		
	
		if (ctx.RIGHT_SHIFT()==null) {
			return visitArith_expr(ctx.arith_expr());
		} else  {
			return createPairFromSingle(new PyShiftRight(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr())), checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr()))));
		}
	}
	
	@Override
	public Pair<Statement, Statement> visitLeft_shift(Left_shiftContext ctx) {
		
		
		
		if (ctx.LEFT_SHIFT()==null) {
			return visitArith_expr(ctx.arith_expr());
		} else  {
			return createPairFromSingle(new PyShiftLeft(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr())), checkAndExtractSingleExpression(visitShift_expr(ctx.shift_expr()))));
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
		
		
		
		if (ctx.arith_expr()==null) {
			return visitTerm(ctx.term());
		} else  {
			return createPairFromSingle(new PyMinus(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitTerm(ctx.term())), checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr()))));
		}
	}
	
	@Override
	public Pair<Statement, Statement> visitAdd(AddContext ctx) {
		
		
		
		if (ctx.arith_expr()==null) {
			return visitTerm(ctx.term());
		} else  {
			return createPairFromSingle(new PyAdd(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitTerm(ctx.term())), checkAndExtractSingleExpression(visitArith_expr(ctx.arith_expr()))));
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
		
		
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyMul(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}	
		
	}
	
	public Pair<Statement, Statement> visitMat_mul(Mat_mulContext ctx) {
		
		
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyMatMul(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}	
		
		
	}
	
	public Pair<Statement, Statement> visitDiv(DivContext ctx) {
		
		
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyDiv(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}	

		
	}
	
	public Pair<Statement, Statement> visitMod(ModContext ctx) {
		
		
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyMod(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
		}
		
	}	
	
	public Pair<Statement, Statement> visitFloorDiv(FloorDivContext ctx) {
		
		
		
		if (ctx.term()==null) {
			return visitFactor(ctx.factor());
		} else  {
			return createPairFromSingle(new PyFloorDiv(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitFactor(ctx.factor())), checkAndExtractSingleExpression(visitTerm(ctx.term()))));
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
		
		
		
		if(ctx.POWER()!=null) {
			return createPairFromSingle(new PyPower(currentCFG, getLocation(ctx), checkAndExtractSingleExpression(visitAtom_expr(ctx.atom_expr())), checkAndExtractSingleExpression(visitFactor(ctx.factor()))));
		}
		else return visitAtom_expr(ctx.atom_expr());
	}

	@Override
	public Pair<Statement, Statement> visitAtom_expr(Atom_exprContext ctx) {
		/*
		atom_expr: (AWAIT)? atom trailer*;
		atom: ('(' (yield_expr|testlist_comp)? ')' |
			   '[' (testlist_comp)? ']' |
			   '{' (dictorsetmaker)? '}' |
			   NAME | NUMBER | STRING+ | '...' | 'None' | 'True' | 'False');
		 */
		if(ctx.AWAIT()!=null)
			throw new UnsupportedStatementException("await is not supported");
		if(ctx.trailer().size()>0) {
			// trailer: '(' (arglist)? ')' | '[' subscriptlist ']' | '.' NAME;
			Expression access = checkAndExtractSingleExpression(visitAtom(ctx.atom()));
			String last_name = access instanceof VariableRef ? ((VariableRef) access).getName() : null;
			Expression previous_access = null;
			for(TrailerContext expr : ctx.trailer()) {
				Statement result;
				if (expr.NAME()!=null) {
					last_name = expr.NAME().getSymbol().getText();
					Global fieldName = new Global(last_name);
					previous_access = access;
					access = new AccessUnitGlobal(currentCFG, getLocation(expr), access, fieldName);
				}
				else if(expr.OPEN_PAREN()!=null) {
					if(last_name==null)
						throw new UnsupportedStatementException("When invoking a method we need to have always the name before the parentheses");
					List<Expression> pars = new ArrayList<>();
					boolean instance = access instanceof AccessUnitGlobal;
					String method_name = last_name;
					if(instance)
						pars.add(previous_access);
					if(expr.arglist()!=null)
						for(ArgumentContext arg : expr.arglist().argument())
							pars.add(checkAndExtractSingleExpression(visitArgument(arg)));

					access = new UnresolvedCall(currentCFG, getLocation(expr), UnresolvedCall.ResolutionStrategy.DYNAMIC_TYPES, instance, method_name, pars.toArray(new Expression[pars.size()]));
					last_name = null;
					previous_access = null;
				}
				else if(expr.OPEN_BRACK()!=null) {
					previous_access = access;
					last_name = null;
					access = new PyArrayAccess(
							access,
							extractExpressionsFromSubscriptlist(expr.subscriptlist()),
							currentCFG,
							getLocation(expr)
					);
				}
				else throw new UnsupportedStatementException();
			}
			return createPairFromSingle(access);
		}
		else return visitAtom(ctx.atom());
	}

	@Override
	public Pair<Statement, Statement> visitAtom(AtomContext ctx) {
		
		
		
		if (ctx.NAME() != null) {
			//crete a variable
			return createPairFromSingle(new VariableRef(currentCFG, ctx.NAME().getText()));
		} else if (ctx.NUMBER() != null) {
			//create a literal int
			return createPairFromSingle(new Literal(currentCFG, ctx.NUMBER().getText(), PyIntType.INSTANCE));
		}else if (ctx.FALSE() != null) {
			//create a literal false
			return createPairFromSingle(new PyFalseLiteral(currentCFG, getLocation(ctx)));
		}else if (ctx.TRUE()!= null) {
			//create a literal true
			return createPairFromSingle(new PyTrueLiteral(currentCFG, getLocation(ctx)));
		}else if (ctx.NONE() != null) {
			//create a literal false
			return createPairFromSingle(new PyNoneLiteral(currentCFG, getLocation(ctx)));
		}
		else if (ctx.STRING().size() > 0) {
			//create a string
			return createPairFromSingle(new PyStringLiteral(currentCFG,getLocation(ctx), ctx.STRING(0).getText()));
		} else if(ctx.yield_expr()!=null) {
			return visitYield_expr(ctx.yield_expr());
		}else if(ctx.dictorsetmaker()!=null) {
			return visitDictorsetmaker(ctx.dictorsetmaker());
		} else if(ctx.OPEN_BRACK()!=null) {
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			ListCreation r = new ListCreation(sts, currentCFG, getLocation(ctx));
			return createPairFromSingle(r);
		} else if(ctx.OPEN_PAREN()!=null) {
			if(ctx.yield_expr()!=null)
				throw new UnsupportedStatementException("yield expressions not supported");
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			TupleCreation r = new TupleCreation(sts, currentCFG, getLocation(ctx));
			return createPairFromSingle(r);
		}
		throw new UnsupportedStatementException();
	}
	private List<Expression> extractExpressionsFromTestlist(TestlistContext ctx) {
		return extractExpressionsFromListOfTests(ctx.test());
	}

	private List<Expression> extractExpressionsFromExprlist(ExprlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if(ctx.expr().size()==0)
			return result;
		for(ExprContext e : ctx.expr())
			result.add(checkAndExtractSingleExpression(visitExpr(e)));
		return result;
	}

	private List<Expression> extractExpressionsFromYieldArg(Yield_argContext ctx) {
		if(ctx.test()!=null) {
			List<Expression> r = new ArrayList<>();
			r.add(checkAndExtractSingleExpression(visitTest(ctx.test())));
			return r;
		}
		else return extractExpressionsFromTestlist(ctx.testlist());
	}

	private List<Expression> extractExpressionsFromListOfTests(List<TestContext> l) {
		List<Expression> result = new ArrayList<>();
		if(l.size()==0)
			return result;
		for(TestContext e : l)
			result.add(checkAndExtractSingleExpression(visitTest(e)));
		return result;
	}

	private List<Expression> extractExpressionsFromSubscriptlist(SubscriptlistContext ctx) {
		List<Expression> result = new ArrayList<>();
		if(ctx.subscript().size()==0)
			return result;
		for(SubscriptContext e : ctx.subscript())
			result.add(checkAndExtractSingleExpression(visitSubscript(e)));
		return result;
	}

	private List<Expression> extractExpressionsFromTestlist_comp(Testlist_compContext ctx) {
		List<Expression> result = new ArrayList<>();
		if(ctx==null || ctx.testOrStar()==null || ctx.testOrStar().size()==0)
			return result;
		for(TestOrStarContext e : ctx.testOrStar())
			result.add(checkAndExtractSingleExpression(visitTestOrStar(e)));
		return result;
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
		if(ctx.COLON()!=null) {
			Expression left = ctx.test1()==null ? null : checkAndExtractSingleExpression(visitTest(ctx.test1().test()));
			Expression right = ctx.test2()==null ? null : checkAndExtractSingleExpression(visitTest(ctx.test2().test()));
			return createPairFromSingle(new RangeValue(left, right, currentCFG, getLocation(ctx)));
		}
		else return visitTest(ctx.test());
	}

	@Override
	public Pair<Statement, Statement> visitSliceop(SliceopContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitDictorsetmaker(DictorsetmakerContext ctx) {
		if(ctx.COLON().size()==0) {
			List<Expression> values = new ArrayList<Expression>();
			for(TestContext exp : ctx.test())
				values.add(checkAndExtractSingleExpression(visitTest(exp)));
			return createPairFromSingle(new SetCreation(values, currentCFG, getLocation(ctx)));
		}
		else {
			throw new UnsupportedStatementException();
		}
	}

	@Override
	public Pair<Statement, Statement> visitClassdef(ClassdefContext ctx) {
		Unit previous = this.currentUnit;
		String name = ctx.NAME().getSymbol().getText();
		this.currentUnit = new PythonUnit(getLocation(ctx), name);
		parseClassBody(ctx.suite());
		this.parsedUnits.add(this.currentUnit);
		this.currentUnit = previous;
		return null;
	}

	private void parseClassBody(SuiteContext ctx) {
		List<Pair<VariableRef, Expression>> fields_init = new ArrayList<>();
		if(ctx.simple_stmt()!=null)
			throw new UnsupportedStatementException("Inside the body of a class we should have only field and method definitions");
		for(StmtContext stmt : ctx.stmt()) {
			if(stmt.simple_stmt()!=null) {
				Pair<VariableRef, Expression> p = parseField(stmt.simple_stmt());
				currentUnit.addGlobal(new Global(p.getLeft().getName()));
				if(p.getRight()!=null)
					fields_init.add(p);
			}
			else if(stmt.compound_stmt().funcdef()!=null) {
				parseMethod(stmt.compound_stmt().funcdef());
			}
			else if(stmt.compound_stmt().decorated()!=null) {
				log.warn("Ignoring decorator "+stmt.compound_stmt().decorated().decorators().getText()+" at code location "+getLocation(stmt));
				DecoratedContext c = stmt.compound_stmt().decorated();
				if(c.funcdef()!=null)
					parseMethod(c.funcdef());
				else if(c.classdef()!=null)
					this.visitClassdef(c.classdef());
				else throw new UnsupportedStatementException("We support only decorated classes and methods");
			}
			else throw new UnsupportedStatementException("Inside the body of a class we should have only field and method definitions");
		}
		dumpConstructor(fields_init, getLocation(ctx));
	}

	private void dumpConstructor(List<Pair<VariableRef, Expression>> fields_init, CodeLocation location) {
		if(fields_init.size()>0) {
			CFG oldCFG = currentCFG;
			currentCFG = new CFG(new CFGDescriptor(currentUnit, true, "<init>"));
			cfgs.add(currentCFG);
			Statement previous = null;
			for (Pair<VariableRef, Expression> init : fields_init) {
				Statement f = new Assignment(currentCFG, init.getLeft().getLocation(), init.getLeft(), init.getRight());
				currentCFG.addNode(f, previous==null);
				if(previous!=null)
					currentCFG.addEdge(new SequentialEdge(previous, f));
				previous = f;
			}
			currentUnit.addCFG(currentCFG);
			currentCFG = oldCFG;
		}

	}

	private Pair<VariableRef, Expression>  parseField(Simple_stmtContext st) {
		try {
			Statement result = checkAndExtractSingleStatement(visitSimple_stmt(st));
			if(result instanceof Assignment) {
				Assignment ass = (Assignment) result;
				Expression assigned = ass.getLeft();
				Expression expr = ass.getRight();
				if(assigned instanceof VariableRef)
					return Pair.of((VariableRef) assigned, expr);
			}
			else if(result instanceof VariableRef)
				return Pair.of((VariableRef) result, null);
			throw new UnsupportedStatementException("Only variables or assignments of variable are supported as field declarations");
		}
		catch(UnsupportedStatementException e) {
			throw new UnsupportedStatementException("Only variables or assignments of variable are supported as field declarations", e);
		}
	}
	@Override
	public Pair<Statement, Statement> visitArglist(ArglistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Pair<Statement, Statement> visitArgument(ArgumentContext ctx) {
		if(ctx.ASSIGN()!=null) {
			Statement target = checkAndExtractSingleStatement(visitTest(ctx.test(0)));
			Statement expression = checkAndExtractSingleStatement(visitTest(ctx.test(1)));
			if((! (target instanceof Expression)) || (! (expression instanceof Expression)))
				throw new UnsupportedStatementException("Assignments require expression both in the left and in the right hand side");
			return createPairFromSingle(new Assignment(currentCFG,  (Expression) target,  (Expression) expression));
		}
		else
			if(ctx.comp_for()!=null || ctx.POWER()!=null || ctx.STAR()!=null || ctx.test().size()!=1)
				throw new UnsupportedStatementException("We support only simple arguments in method calls");
		else return visitTest(ctx.test(0));
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