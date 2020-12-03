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
import java.util.List;

import org.antlr.v4.runtime.CharStreams;
import org.antlr.v4.runtime.CommonTokenStream;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.misc.Pair;
import org.antlr.v4.runtime.tree.ParseTree;
import org.antlr.v4.runtime.tree.ParseTreeWalker;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import it.unive.lisa.cfg.CFG;
import it.unive.lisa.cfg.CFGDescriptor;
import it.unive.lisa.cfg.edge.FalseEdge;
import it.unive.lisa.cfg.edge.SequentialEdge;
import it.unive.lisa.cfg.edge.TrueEdge;
import it.unive.lisa.cfg.statement.NoOp;
import it.unive.lisa.cfg.statement.Parameter;
import it.unive.lisa.cfg.statement.Statement;
import it.unive.lisa.logging.IterationLogger;
import it.unive.pylisa.antlr.Python3BaseVisitor;
import it.unive.pylisa.antlr.Python3Lexer;
import it.unive.pylisa.antlr.Python3Parser;
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
import it.unive.pylisa.antlr.Python3Parser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Not_testContext;
import it.unive.pylisa.antlr.Python3Parser.Or_testContext;
import it.unive.pylisa.antlr.Python3Parser.ParametersContext;
import it.unive.pylisa.antlr.Python3Parser.Pass_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.PowerContext;
import it.unive.pylisa.antlr.Python3Parser.Raise_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Return_stmtContext;
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
import it.unive.pylisa.antlr.Python3Visitor;



public class PyToCFG<T> extends Python3BaseVisitor<T>{
	
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
	 * Builds an instance of @PyToCFG for a given Python program
	 * given at the location filePath.
	 *  
	 * @param filePath file path to a Python program.
	 */
	public PyToCFG(String filePath) {
		this.cfgs = new HashSet<CFG>();
		this.filePath = filePath;
	}

	/**
	 * Returns the parsed file path.
	 * @return the parsed file path
	 */
	public String getFilePath() {
		return filePath;
	}

	/**
	 * Returns the parsed CFGs
	 * @return the parsed CFGs
	 */
	public Collection<CFG> getCFGs() {
		return cfgs;
	}

	/**
	 * Current CFG to parse
	 */
	private CFG currentCFG;

	public static void main(String[] args) throws IOException {
		String file = "src/test/resources/pyTest/py1.py";
		PyToCFG translator = new PyToCFG(file);
		System.err.println(translator.toLiSACFG()); //.iterator().next().getEdges()
	}

	/**
	 * Returns the collection of @CFG in a Python program at filePath.
	 * 
	 * @return collection of @CFG in file
	 * @throws IOException if {@code stream} to file cannot be written to or closed
	 */
	public Python3Parser p;
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
		p = parser;
		ParseTree tree = parser.file_input();
				
		visit(tree);
		log.info(tree.getClass());
		log.info(tree.toStringTree(parser));
		
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

		Parameter[] cfgArgs = new Parameter[]{};

			
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
		log.info(ctx.getText());
		Object result = visitChildren(ctx);
		/*if (!(result instanceof Pair<?,?>))
			throw new IllegalStateException("Pair of Statements expected");
		else */
			return (T) result;
	}

	@Override
	public T visitSimple_stmt(Simple_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitSimple_stmt(ctx);
	}

	@Override
	public T visitSmall_stmt(Small_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitSmall_stmt(ctx);
	}

	@Override
	public T visitExpr_stmt(Expr_stmtContext ctx) {
		// TODO Auto-generated method stub
		return super.visitExpr_stmt(ctx);
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
	public  T visitIf_stmt(If_stmtContext ctx) {


		
		log.info("Sono nell'if");
		// Visit if statement Boolean Guard
		//visitTest(ctx.test(0));
		
		//log.info("test");
		//visitSuite(ctx.suite(0));
		

		// Visit if statement Boolean Guard
		Statement booleanGuard = (Statement) visitTest(ctx.test(0));
		currentCFG.addNode(booleanGuard);
		
		Statement x = new Statement(currentCFG,"ciao",0,0);

		try {  
            Writer w = new FileWriter("./output.txt");  
            currentCFG.dump(w, "test");
            w.close();
            log.info("Done");  
        } catch (IOException e) {
        	log.info("c'è stato un errore");
            e.printStackTrace();  
        }  
		
		
		NoOp ifExitNode = new NoOp(currentCFG);
		currentCFG.addNode(ifExitNode);
		

		
//		Pair<Statement, Statement> trueBlock = visitSuite(ctx.suite(0));
//		Statement exitStatementTrueBranch = trueBlock.getRight();
//		Statement entryStatementTrueBranch = trueBlock.getLeft();
		
		//if testLenght is >1 the context contains elif
		int testLenght=ctx.test().size();
		
		if (ctx.ELIF() == null && ctx.ELSE() == null) {
			
			// If statement without else and elif branch
//			currentCFG.addEdge(new TrueEdge(booleanGuard, entryStatementTrueBranch));			
//			currentCFG.addEdge(new FalseEdge(booleanGuard, ifExitNode));			
//			currentCFG.addEdge(new SequentialEdge(exitStatementTrueBranch, ifExitNode));
			
		} else {	
			int i=1;
			if (ctx.ELIF() != null) {
				
				//continue visit every elif
				while (ctx.test(i) != null) {
					
					Statement booleanGuardElif = (Statement) visitTest(ctx.test(i));
					currentCFG.addNode(booleanGuardElif);
					currentCFG.addEdge(new FalseEdge(booleanGuard, booleanGuardElif));
//					
//					Pair<Statement, Statement> trueBlockElif = visitSuite(ctx.suite(i));
//					Statement exitStatementTrueBranchElif = trueBlock.getRight();
//					Statement entryStatementTrueBranchElif = trueBlock.getLeft();
					
				}
			}
			
			if(ctx.ELSE() != null) {
				
			}
		}
		
		
		
		if (ctx.ELSE() != null) {
			log.info("Esiste else");
		}
		
		visitSuite(ctx.suite(1));
		

	 	
		
		
		
		return null;
	}

	@Override
	public T visitWhile_stmt(While_stmtContext ctx) {
		log.info("Sono nel while");
		// TODO Auto-generated method stub
		return super.visitWhile_stmt(ctx);
	}

	@Override
	public T visitFor_stmt(For_stmtContext ctx) {
		// TODO Auto-generated method stub
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
		log.info("Sono nel suite");
		log.info(ctx.getText());
		for(StmtContext stmt : ctx.stmt()) {
			Object result = visitStmt(stmt);
			
			/*
			laststmt != null
					entry nuovo aggacia al veccchio
					*/
		}
		return null;
	}

	@Override
	public T visitTest(TestContext ctx) {
		log.info("Sono nel test");
		log.info(ctx.getText());

		return null;
	}

	@Override
	public T visitTest_nocond(Test_nocondContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTest_nocond(ctx);
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
		// TODO Auto-generated method stub
		return super.visitOr_test(ctx);
	}

	@Override
	public T visitAnd_test(And_testContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAnd_test(ctx);
	}

	@Override
	public T visitNot_test(Not_testContext ctx) {
		// TODO Auto-generated method stub
		return super.visitNot_test(ctx);
	}

	@Override
	public T visitComparison(ComparisonContext ctx) {
		// TODO Auto-generated method stub
		return super.visitComparison(ctx);
	}

	@Override
	public T visitComp_op(Comp_opContext ctx) {
		return null;
//		
//		// Python equals (==)
//		if (ctx.EQUALS() != null)
//			return new PyEquals();
//
//		// Python grater (>)
//		if (ctx.GREATER_THAN() != null) 
//			return new PyGreater();
//
//		// Python greater equal (>=)
//		if (ctx.GT_EQ() != null)
//			return new PyGreaterEqual();
//
//		// Python in (in)
//		if (ctx.IN() != null)
//			return new PyIn();
//
//		// Python is (is)
//		if (ctx.IS() != null) 
//			return new PyIs();
//
//		// Python less (<)
//		if (ctx.LESS_THAN() != null)
//			return new PyLess();
//
//		// Python less equal (<=)
//		if (ctx.LT_EQ() != null)
//			return new PyLessEqual();
//
//		// Python not (not)
//		if (ctx.NOT() != null)
//			return new PyNot();
//
//		// Python not equals (<>)
//		if (ctx.NOT_EQ_1() != null)
//			return new PyNot1();
//
//		// Python not equals (!=)
//		if (ctx.NOT_EQ_2() != null)
//			return new PyNot2();
//
//		Object child = visitChildren(ctx);
//		if (!(child instanceof Comp_op))
//			throw new IllegalStateException("Comp_op expected, found Statement instead");
//		else
//			return (Comp_op) child;
	}

	@Override
	public T visitStar_expr(Star_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitStar_expr(ctx);
	}

	@Override
	public T visitExpr(ExprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitExpr(ctx);
	}

	@Override
	public T visitXor_expr(Xor_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitXor_expr(ctx);
	}

	@Override
	public T visitAnd_expr(And_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitAnd_expr(ctx);
	}

	@Override
	public T visitShift_expr(Shift_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitShift_expr(ctx);
	}

	@Override
	public T visitArith_expr(Arith_exprContext ctx) {
		// TODO Auto-generated method stub
		return super.visitArith_expr(ctx);
	}

	@Override
	public T visitTerm(TermContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTerm(ctx);
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
		// TODO Auto-generated method stub
		return super.visitExprlist(ctx);
	}

	@Override
	public T visitTestlist(TestlistContext ctx) {
		// TODO Auto-generated method stub
		return super.visitTestlist(ctx);
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