package it.unive.pylisa.frontend.statement;

import it.unive.lisa.logging.IterationLogger;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.controlFlow.IfThenElse;
import it.unive.lisa.program.cfg.controlFlow.Loop;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.FalseEdge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.edge.TrueEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.Async_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Compound_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Except_clauseContext;
import it.unive.pylisa.antlr.Python3Parser.File_inputContext;
import it.unive.pylisa.antlr.Python3Parser.For_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.If_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.SuiteContext;
import it.unive.pylisa.antlr.Python3Parser.Try_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.While_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.With_itemContext;
import it.unive.pylisa.antlr.Python3Parser.With_stmtContext;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.PyAddition;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.frontend.expression.DunderMethods;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles Python compound control-flow statements — if/while/for/try/with plus
 * the {@code file_input} top-level production, the {@code compound_stmt}
 * wrapper, and the {@code suite} helper. Extracted from
 * {@link StatementVisitor} in Chunk 4.
 */
public final class ControlFlowVisitor {

	private static final Logger LOG = LogManager.getLogger(ControlFlowVisitor.class);

	private final ParserContext ctx;
	private final ParserSupport support;

	public ControlFlowVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public PyCFG visitFile_input(
			File_inputContext pctx) {
		ctx.currentCFG(new PyCFG(support.buildInitModuleCFGDescriptor(support.getLocation(pctx))));
		ImportModule builtinsModule = new ImportModule(ctx.currentCFG(), support.getLocation(pctx), "builtins",
				PyModuleType.lookup("builtins").getUnit());
		ctx.currentCFG().addNode(builtinsModule, true);
		ctx.cfs(new HashSet<>());
		ctx.currentModule().addCodeMember(ctx.currentCFG());
		Expression targetName = new PythonScopedAttributeAccessRef(ctx.currentCFG(), support.getLocation(pctx),
				ctx.currentModule(),
				new Global(support.getLocation(pctx), ctx.currentModule(), "__name__", false));

		PyAssign nameAssign = new PyAssign(ctx.currentCFG(), support.getLocation(pctx), targetName,
				new StringLiteral(ctx.currentCFG(), support.getLocation(pctx), ctx.currentModule().getName()));
		ctx.currentCFG().addNode(nameAssign);
		ctx.currentCFG().addEdge(new SequentialEdge(builtinsModule, nameAssign));
		Statement lastStmt = nameAssign;
		for (StmtContext stmt : IterationLogger.iterate(LOG, pctx.stmt(), "Parsing stmt lists...", "Global stmt")) {
			Object visited;
			try {
				if (stmt.compound_stmt() != null)
					visited = visitCompound_stmt(stmt.compound_stmt());
				else
					visited = ctx.stmt().visitSimple_stmt(stmt.simple_stmt());
			} catch (RuntimeException e) {
				if (!ctx.continueOnUnsupportedStatement())
					throw e;
				LOG.warn("[PyLiSA] Skipping unsupported statement at "
						+ support.getLocation(stmt) + ": " + e.getClass().getSimpleName()
						+ ": " + e.getMessage());
				continue;
			}

			if (!(visited instanceof Triple<?, ?, ?>))
				continue;

			@SuppressWarnings("unchecked")
			Triple<Statement, NodeList<CFG, Statement, Edge>,
					Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) visited;
			ctx.currentCFG().getNodeList().mergeWith(st.getMiddle());
			if (lastStmt == null)
				ctx.currentCFG().getEntrypoints().add(st.getLeft());
			else if (!lastStmt.stopsExecution())
				ctx.currentCFG().addEdge(new SequentialEdge(lastStmt, st.getLeft()));
			lastStmt = st.getRight();
		}

		support.addRetNodesToCurrentCFG();
		ctx.cfs().forEach(ctx.currentCFG().getDescriptor()::addControlFlowStructure);
		ctx.currentCFG().simplify();
		return ctx.currentCFG();
	}

	public Object visitCompound_stmt(
			Compound_stmtContext pctx) {
		if (pctx.funcdef() != null)
			return ctx.def().visitFuncdef(pctx.funcdef());
		else if (pctx.if_stmt() != null)
			return visitIf_stmt(pctx.if_stmt());
		else if (pctx.while_stmt() != null)
			return visitWhile_stmt(pctx.while_stmt());
		else if (pctx.for_stmt() != null)
			return visitFor_stmt(pctx.for_stmt());
		else if (pctx.try_stmt() != null)
			return visitTry_stmt(pctx.try_stmt());
		else if (pctx.with_stmt() != null)
			return visitWith_stmt(pctx.with_stmt());
		else if (pctx.classdef() != null)
			return ctx.def().visitClassdef(pctx.classdef());
		else if (pctx.decorated() != null)
			return ctx.def().visitDecorated(pctx.decorated());
		else if (pctx.async_stmt() != null)
			return visitAsync_stmt(pctx.async_stmt());
		return support.unsupported(pctx, "Statement not yet supported");
	}

	public Object visitAsync_stmt(
			Async_stmtContext pctx) {
		support.unsound(pctx, "async stmt treated as sync");
		if (pctx.funcdef() != null)
			return ctx.def().visitFuncdef(pctx.funcdef());

		if (pctx.for_stmt() != null)
			return visitFor_stmt(pctx.for_stmt());

		if (pctx.with_stmt() != null)
			return visitWith_stmt(pctx.with_stmt());

		throw new UnsupportedStatementException("Expecting with, for, def, in Async_stmtContext.");
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitIf_stmt(
			If_stmtContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Statement booleanGuard = ctx.expr().visitTest(pctx.test(0));
		block.addNode(booleanGuard);

		NoOp ifExitNode = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		block.addNode(ifExitNode);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitSuite(pctx.suite(0));
		block.mergeWith(trueBlock.getMiddle());
		Statement trueEntry = trueBlock.getLeft();
		Statement trueExit = trueBlock.getRight();

		block.addEdge(new TrueEdge(booleanGuard, trueEntry));
		if (!ControlFlowBuilder.stopsExecution(trueExit))
			block.addEdge(new SequentialEdge(trueExit, ifExitNode));

		List<Pair<Statement, Collection<Statement>>> branches = new LinkedList<>();
		int testLength = pctx.test().size();
		Statement lastElifGuard = booleanGuard;
		if (testLength > 1)
			for (int i = 1; i < testLength; i++) {
				Statement elifGuard = ctx.expr().visitTest(pctx.test(i));
				block.addNode(elifGuard);
				block.addEdge(new FalseEdge(lastElifGuard, elifGuard));
				lastElifGuard = elifGuard;
				Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> elifBlock = visitSuite(pctx.suite(i));
				block.mergeWith(elifBlock.getMiddle());
				branches.add(Pair.of(elifGuard, elifBlock.getMiddle().getNodes()));
				Statement elifEntry = elifBlock.getLeft();
				Statement elifExit = elifBlock.getRight();

				block.addEdge(new TrueEdge(elifGuard, elifEntry));
				if (!ControlFlowBuilder.stopsExecution(elifExit))
					block.addEdge(new SequentialEdge(elifExit, ifExitNode));
			}

		Collection<Statement> falseStatements = new HashSet<>();
		if (pctx.ELSE() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitSuite(
					pctx.suite(pctx.suite().size() - 1));
			block.mergeWith(falseBlock.getMiddle());
			falseStatements.addAll(falseBlock.getMiddle().getNodes());
			Statement falseEntry = falseBlock.getLeft();
			Statement falseExit = falseBlock.getRight();

			block.addEdge(new FalseEdge(lastElifGuard, falseEntry));
			if (!ControlFlowBuilder.stopsExecution(falseExit))
				block.addEdge(new SequentialEdge(falseExit, ifExitNode));
		} else {
			if (!ControlFlowBuilder.stopsExecution(lastElifGuard))
				block.addEdge(new FalseEdge(lastElifGuard, ifExitNode));
		}

		for (int k = branches.size() - 1; k >= 0; k--) {
			Pair<Statement, Collection<Statement>> branch = branches.get(k);
			ctx.cfs().add(new IfThenElse(ctx.currentCFG().getNodeList(), branch.getLeft(), ifExitNode,
					branch.getRight(),
					new HashSet<>(falseStatements)));
		}
		ctx.cfs().add(new IfThenElse(ctx.currentCFG().getNodeList(), booleanGuard, ifExitNode,
				trueBlock.getMiddle().getNodes(),
				falseStatements));
		return Triple.of(booleanGuard, block, ifExitNode);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWhile_stmt(
			While_stmtContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		NoOp whileExitNode = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		block.addNode(whileExitNode);

		Statement condition = ctx.expr().visitTest(pctx.test());
		block.addNode(condition);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitSuite(pctx.suite(0));

		block.mergeWith(trueBlock.getMiddle());
		ControlFlowBuilder.rewireLoopBody(block, trueBlock.getMiddle(), condition, whileExitNode);
		block.addEdge(new TrueEdge(condition, trueBlock.getLeft()));
		if (!trueBlock.getRight().stopsExecution())
			block.addEdge(new SequentialEdge(trueBlock.getRight(), condition));

		Statement firstFollower;
		if (pctx.ELSE() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitSuite(pctx.suite(1));
			block.mergeWith(falseBlock.getMiddle());
			block.addEdge(new FalseEdge(condition, falseBlock.getLeft()));
			if (!falseBlock.getRight().stopsExecution())
				block.addEdge(new SequentialEdge(falseBlock.getRight(), whileExitNode));
			firstFollower = falseBlock.getLeft();
		} else {
			block.addEdge(new FalseEdge(condition, whileExitNode));
			firstFollower = whileExitNode;
		}

		ctx.cfs().add(new Loop(ctx.currentCFG().getNodeList(), condition, firstFollower,
				trueBlock.getMiddle().getNodes()));
		return Triple.of(condition, block, whileExitNode);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFor_stmt(
			For_stmtContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		NoOp exit = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		block.addNode(exit);

		List<Expression> exprs = ctx.stmt().visitExprlist(pctx.exprlist());
		Expression variable;
		if (exprs.size() == 1)
			variable = exprs.get(0);
		else
			variable = new TupleCreation(ctx.currentCFG(), support.getLocation(pctx),
					exprs.toArray(Expression[]::new));

		List<Expression> list = ctx.stmt().visitTestlist(pctx.testlist());
		Expression collection = list.iterator().next();

		VariableRef counter = new VariableRef(
				ctx.currentCFG(),
				support.getLocation(pctx),
				"__counter_location" + support.getLocation(pctx).getLine(), Int32Type.INSTANCE);
		Expression[] counter_pars = { collection, counter };

		Assignment counter_init = new Assignment(
				ctx.currentCFG(),
				support.getLocation(pctx),
				counter,
				new Int32Literal(ctx.currentCFG(), support.getLocation(pctx), 0));
		block.addNode(counter_init);

		UnresolvedCall condition = new UnresolvedCall(
				ctx.currentCFG(),
				support.getLocation(pctx),
				CallType.INSTANCE,
				null,
				DunderMethods.LT,
				counter,
				new UnresolvedCall(
						ctx.currentCFG(),
						support.getLocation(pctx),
						CallType.INSTANCE,
						null,
						DunderMethods.LEN,
						LeftToRightEvaluation.INSTANCE,
						collection));
		block.addNode(condition);

		Assignment element_assignment = new Assignment(
				ctx.currentCFG(),
				support.getLocation(pctx),
				variable,
				new UnresolvedCall(
						ctx.currentCFG(),
						support.getLocation(pctx),
						CallType.INSTANCE,
						null,
						DunderMethods.GETITEM,
						LeftToRightEvaluation.INSTANCE,
						counter_pars));
		block.addNode(element_assignment);

		Assignment counter_increment = new Assignment(
				ctx.currentCFG(),
				support.getLocation(pctx),
				counter,
				new PyAddition(
						ctx.currentCFG(),
						support.getLocation(pctx),
						counter,
						new Int32Literal(
								ctx.currentCFG(),
								support.getLocation(pctx),
								1)));
		block.addNode(counter_increment);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> body = visitSuite(pctx.suite(0));
		block.mergeWith(body.getMiddle());

		ControlFlowBuilder.rewireLoopBody(block, body.getMiddle(), condition, exit);

		block.addEdge(new SequentialEdge(counter_init, condition));
		block.addEdge(new TrueEdge(condition, element_assignment));
		block.addEdge(new SequentialEdge(element_assignment, body.getLeft()));
		if (!body.getRight().stopsExecution())
			block.addEdge(new SequentialEdge(body.getRight(), counter_increment));
		block.addEdge(new SequentialEdge(counter_increment, condition));
		block.addEdge(new FalseEdge(condition, exit));

		Collection<Statement> nodes = new HashSet<>(body.getMiddle().getNodes());
		nodes.add(element_assignment);
		nodes.add(counter_increment);
		ctx.cfs().add(new Loop(ctx.currentCFG().getNodeList(), condition, exit, nodes));
		return Triple.of(counter_init, block, exit);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitTry_stmt(
			Try_stmtContext pctx) {
		support.unsound(pctx, "try block: except clauses conservatively modeled as a bypass");
		// Model `try: BODY except: …` as "BODY may execute fully OR an
		// exception may be caught anywhere inside, in which case control
		// resumes at the try-exit carrying the pre-try state".
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> body = visitSuite(pctx.suite(0));
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		NoOp entry = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		NoOp exit = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		block.addNode(entry);
		block.addNode(exit);
		block.mergeWith(body.getMiddle());
		block.addEdge(new SequentialEdge(entry, body.getLeft()));
		if (body.getRight() != null && !body.getRight().stopsExecution())
			block.addEdge(new SequentialEdge(body.getRight(), exit));
		block.addEdge(new SequentialEdge(entry, exit));
		return Triple.of(entry, block, exit);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_stmt(
			With_stmtContext pctx) {
		int withSize = pctx.with_item().size();
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> curr = visitWith_item(pctx.with_item(0));
		Statement start = curr.getLeft();
		Statement prev = curr.getRight();
		block.mergeWith(curr.getMiddle());

		for (int i = 1; i < withSize; i++) {
			curr = visitWith_item(pctx.with_item(i));
			block.mergeWith(curr.getMiddle());
			block.addEdge(new SequentialEdge(prev, curr.getLeft()));
			prev = curr.getRight();
		}

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> suite = visitSuite(pctx.suite());
		block.mergeWith(suite.getMiddle());
		block.addEdge(new SequentialEdge(prev, suite.getLeft()));

		return Triple.of(start, block, suite.getRight());
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_item(
			With_itemContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Statement test = ctx.expr().visitTest(pctx.test());
		block.addNode(test);
		Statement expr = test;
		if (pctx.expr() != null) {
			expr = ctx.expr().visitExpr(pctx.expr());
			block.addNode(expr);
			block.addEdge(new SequentialEdge(test, expr));
		}
		return Triple.of(test, block, expr);
	}

	public Object visitExcept_clause(
			Except_clauseContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSuite(
			SuiteContext pctx) {
		if (pctx.simple_stmt() != null)
			return ctx.stmt().visitSimple_stmt(pctx.simple_stmt());
		else {
			NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
			Statement last = null, first = null;
			for (StmtContext element : pctx.stmt()) {
				Object parsed = ctx.stmt().visitStmt(element);
				if (!(parsed instanceof Triple<?, ?, ?>))
					continue;

				@SuppressWarnings("unchecked")
				Triple<Statement, NodeList<CFG, Statement, Edge>,
						Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) parsed;
				block.mergeWith(st.getMiddle());
				if (first == null)
					first = st.getLeft();
				if (last != null && !last.stopsExecution())
					block.addEdge(new SequentialEdge(last, st.getLeft()));
				last = st.getRight();
			}
			return Triple.of(first, block, last);
		}
	}
}
