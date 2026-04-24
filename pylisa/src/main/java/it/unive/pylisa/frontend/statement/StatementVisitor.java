package it.unive.pylisa.frontend.statement;

import it.unive.lisa.logging.IterationLogger;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
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
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.AnnassignContext;
import it.unive.pylisa.antlr.Python3Parser.Assert_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Async_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.AugassignContext;
import it.unive.pylisa.antlr.Python3Parser.Break_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Compound_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Continue_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Del_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_as_namesContext;
import it.unive.pylisa.antlr.Python3Parser.Dotted_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Except_clauseContext;
import it.unive.pylisa.antlr.Python3Parser.ExprContext;
import it.unive.pylisa.antlr.Python3Parser.Expr_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.ExprlistContext;
import it.unive.pylisa.antlr.Python3Parser.File_inputContext;
import it.unive.pylisa.antlr.Python3Parser.Flow_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.For_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Global_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.If_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_as_namesContext;
import it.unive.pylisa.antlr.Python3Parser.Import_fromContext;
import it.unive.pylisa.antlr.Python3Parser.Import_nameContext;
import it.unive.pylisa.antlr.Python3Parser.Import_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Pass_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Raise_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Return_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Simple_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Small_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.SuiteContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
import it.unive.pylisa.antlr.Python3Parser.TestlistContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_star_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Try_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.While_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.With_itemContext;
import it.unive.pylisa.antlr.Python3Parser.With_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_stmtContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.Break;
import it.unive.pylisa.cfg.expression.Continue;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAccessInstanceGlobal;
import it.unive.pylisa.cfg.expression.PyAddition;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.statement.FromImport;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles Python statements and the top-level {@code file_input} production.
 * One of three sibling category visitors introduced in Chunk 2; shares state
 * with {@link ParserContext} and cross-dispatches via {@code ctx.expr()} /
 * {@code ctx.def()}.
 */
public final class StatementVisitor extends Python3ParserBaseVisitor<Object> {

	private static final Logger LOG = LogManager.getLogger(StatementVisitor.class);

	private final ParserContext ctx;
	private final ParserSupport support;

	public StatementVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	@Override
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
					visited = visitSimple_stmt(stmt.simple_stmt());
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

	@Override
	public Object visitStmt(
			StmtContext pctx) {
		if (pctx.simple_stmt() != null)
			return visitSimple_stmt(pctx.simple_stmt());
		else
			return visitCompound_stmt(pctx.compound_stmt());
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSimple_stmt(
			Simple_stmtContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		Statement first = null, last = null;
		for (int i = 0; i < pctx.small_stmt().size(); i++) {
			Statement st = visitSmall_stmt(pctx.small_stmt(i));
			block.addNode(st);
			if (first == null)
				first = st;
			if (last != null && !last.stopsExecution())
				block.addEdge(new SequentialEdge(last, st));
			last = st;
		}

		return Triple.of(first, block, last);
	}

	@Override
	public Statement visitSmall_stmt(
			Small_stmtContext pctx) {
		if (pctx.expr_stmt() != null)
			return visitExpr_stmt(pctx.expr_stmt());
		else if (pctx.del_stmt() != null)
			return visitDel_stmt(pctx.del_stmt());
		else if (pctx.pass_stmt() != null)
			return visitPass_stmt(pctx.pass_stmt());
		else if (pctx.import_stmt() != null)
			return visitImport_stmt(pctx.import_stmt());
		else if (pctx.assert_stmt() != null)
			return visitAssert_stmt(pctx.assert_stmt());
		else if (pctx.flow_stmt() != null)
			return visitFlow_stmt(pctx.flow_stmt());
		else if (pctx.nonlocal_stmt() != null)
			return new NoOp(ctx.currentCFG(), support.getLocation(pctx)); // TODO
		else if (pctx.global_stmt() != null)
			return new NoOp(ctx.currentCFG(), support.getLocation(pctx)); // TODO
		throw new UnsupportedStatementException("Simple statement not yet supported");
	}

	@Override
	public Expression visitExpr_stmt(
			Expr_stmtContext pctx) {
		if (pctx.annassign() != null) {
			boolean oldPrepend = ctx.shouldPrependUnitAccess();
			ctx.shouldPrependUnitAccess(false);
			Expression rawTarget = visitTestlist_star_expr(pctx.testlist_star_expr(0));
			ctx.shouldPrependUnitAccess(oldPrepend);

			declareAssignedNames(rawTarget);
			Expression target = scopeAssignmentTarget(rawTarget);
			if (pctx.annassign().ASSIGN() != null && pctx.annassign().test().size() > 1)
				return new PyAssign(ctx.currentCFG(), support.getLocation(pctx), target,
						ctx.expr().visitTest(pctx.annassign().test(1)));

			return target;
		}

		if (pctx.ASSIGN().size() == 0)
			if (pctx.testlist_star_expr().size() != 1)
				throw new UnsupportedStatementException();
			else
				return visitTestlist_star_expr(pctx.testlist_star_expr(0));

		boolean oldPrepend = ctx.shouldPrependUnitAccess();
		ctx.shouldPrependUnitAccess(false);
		Expression rawTarget = visitTestlist_star_expr(pctx.testlist_star_expr(0));
		ctx.shouldPrependUnitAccess(oldPrepend);

		declareAssignedNames(rawTarget);
		Expression target = scopeAssignmentTarget(rawTarget);

		// Subscript write: d["key"] = value → d.__setitem__("key", value)
		// (instead of PyAssign(d.__getitem__("key"), value) which crashes LiSA)
		if (rawTarget instanceof FunctionApply fa
				&& fa.getSubExpressions().length == 3
				&& fa.getSubExpressions()[0] instanceof AttributeAccess aa
				&& "__getitem__".equals(aa.getTarget())) {
			Expression receiver = fa.getSubExpressions()[1];
			Expression key = fa.getSubExpressions()[2];
			Expression rhs = visitTestlist_star_expr(pctx.testlist_star_expr(1));
			Expression setitemAttr = new AttributeAccess(
					ctx.currentCFG(), support.getLocation(pctx), receiver, "__setitem__");
			return new FunctionApply(ctx.currentCFG(), support.getLocation(pctx), setitemAttr,
					new Expression[] { receiver, key, rhs }, true);
		}

		return new PyAssign(ctx.currentCFG(), support.getLocation(pctx),
				target,
				visitTestlist_star_expr(pctx.testlist_star_expr(1)));
	}

	@Override
	public Expression visitTestlist_star_expr(
			Testlist_star_exprContext pctx) {
		if (pctx.test().size() == 1)
			return ctx.expr().visitTest(pctx.test(0));

		support.unsound(pctx, "tuple creation treated as first element");
		return ctx.expr().visitTest(pctx.test(0));
	}

	@Override
	public Object visitAnnassign(
			AnnassignContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitAugassign(
			AugassignContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitDel_stmt(
			Del_stmtContext pctx) {
		if (pctx.exprlist().star_expr().size() > 0)
			return support.unsupported(pctx, "We support only expressions without * in del statements");
		return new UnresolvedCall(
				ctx.currentCFG(),
				support.getLocation(pctx),
				CallType.STATIC,
				Program.PROGRAM_NAME,
				"del",
				LeftToRightEvaluation.INSTANCE,
				visitExprlist(pctx.exprlist()).toArray(new Expression[pctx.exprlist().expr().size()]));
	}

	@Override
	public Statement visitPass_stmt(
			Pass_stmtContext pctx) {
		return new NoOp(ctx.currentCFG(), support.getLocation(pctx));
	}

	@Override
	public Object visitGlobal_stmt(
			Global_stmtContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitNonlocal_stmt(
			Nonlocal_stmtContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitAssert_stmt(
			Assert_stmtContext pctx) {
		return new UnresolvedCall(
				ctx.currentCFG(),
				support.getLocation(pctx),
				CallType.STATIC,
				"assert",
				Program.PROGRAM_NAME,
				LeftToRightEvaluation.INSTANCE,
				visitTestlist(pctx.testlist()).toArray(new Expression[pctx.testlist().test().size()]));
	}

	@Override
	public Statement visitFlow_stmt(
			Flow_stmtContext pctx) {
		if (pctx.return_stmt() != null)
			return visitReturn_stmt(pctx.return_stmt());

		if (pctx.raise_stmt() != null) {
			support.unsound(pctx, "raise treated as no-op");
			return new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		}

		if (pctx.yield_stmt() != null) {
			Yield_argContext yieldArg = pctx.yield_stmt().yield_expr().yield_arg();
			if (yieldArg == null) {
				return new NoOp(ctx.currentCFG(), support.getLocation(pctx));
			}
			List<Expression> l = ctx.expr().extractExpressionsFromYieldArg(yieldArg);
			return new UnresolvedCall(
					ctx.currentCFG(),
					support.getLocation(pctx),
					CallType.STATIC,
					Program.PROGRAM_NAME,
					"yield from",
					LeftToRightEvaluation.INSTANCE,
					l.toArray(new Expression[0]));
		}

		if (pctx.continue_stmt() != null)
			return visitContinue_stmt(pctx.continue_stmt());

		if (pctx.break_stmt() != null)
			return visitBreak_stmt(pctx.break_stmt());

		throw new UnsupportedStatementException();
	}

	@Override
	public Statement visitReturn_stmt(
			Return_stmtContext pctx) {
		if (pctx.testlist() == null)
			return new Ret(ctx.currentCFG(), support.getLocation(pctx));
		if (pctx.testlist().test().size() == 1)
			return new Return(ctx.currentCFG(), support.getLocation(pctx),
					ctx.expr().visitTest(pctx.testlist().test(0)));
		else {
			support.unsound(pctx, "multiple return values treated as first value");
			return new Return(ctx.currentCFG(), support.getLocation(pctx),
					ctx.expr().visitTest(pctx.testlist().test(0)));
		}
	}

	@Override
	public Statement visitBreak_stmt(
			Break_stmtContext pctx) {
		return new Break(ctx.currentCFG(), support.getLocation(pctx));
	}

	@Override
	public Statement visitContinue_stmt(
			Continue_stmtContext pctx) {
		return new Continue(ctx.currentCFG(), support.getLocation(pctx));
	}

	@Override
	public Object visitYield_stmt(
			Yield_stmtContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitRaise_stmt(
			Raise_stmtContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
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

	@Override
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

	@Override
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
		if (!trueExit.stopsExecution() && !(trueExit instanceof Continue) && !(trueExit instanceof Break))
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
				if (!elifExit.stopsExecution() && !(elifExit instanceof Continue) && !(elifExit instanceof Break))
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
			if (!falseExit.stopsExecution() && !(falseExit instanceof Continue) && !(falseExit instanceof Break))
				block.addEdge(new SequentialEdge(falseExit, ifExitNode));
		} else {
			if (!lastElifGuard.stopsExecution() && !(lastElifGuard instanceof Continue)
					&& !(lastElifGuard instanceof Break))
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

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWhile_stmt(
			While_stmtContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		NoOp whileExitNode = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		block.addNode(whileExitNode);

		Statement condition = ctx.expr().visitTest(pctx.test());
		block.addNode(condition);

		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> trueBlock = visitSuite(pctx.suite(0));

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

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFor_stmt(
			For_stmtContext pctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
		NoOp exit = new NoOp(ctx.currentCFG(), support.getLocation(pctx));
		block.addNode(exit);

		List<Expression> exprs = visitExprlist(pctx.exprlist());
		Expression variable;
		if (exprs.size() == 1)
			variable = exprs.get(0);
		else
			variable = new TupleCreation(ctx.currentCFG(), support.getLocation(pctx),
					exprs.toArray(Expression[]::new));

		List<Expression> list = visitTestlist(pctx.testlist());
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
				"__lt__",
				counter,
				new UnresolvedCall(
						ctx.currentCFG(),
						support.getLocation(pctx),
						CallType.INSTANCE,
						null,
						"__len__",
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
						"__getitem__",
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

	@Override
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

	@Override
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

	@Override
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

	@Override
	public Object visitExcept_clause(
			Except_clauseContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSuite(
			SuiteContext pctx) {
		if (pctx.simple_stmt() != null)
			return visitSimple_stmt(pctx.simple_stmt());
		else {
			NodeList<CFG, Statement, Edge> block = new NodeList<>(ParserContext.SEQUENTIAL_SINGLETON);
			Statement last = null, first = null;
			for (StmtContext element : pctx.stmt()) {
				Object parsed = visitStmt(element);
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

	@Override
	public List<Expression> visitTestlist(
			TestlistContext pctx) {
		List<Expression> result = new ArrayList<>(pctx.test().size());
		if (pctx.test().size() == 0)
			return result;
		for (TestContext e : pctx.test())
			result.add(ctx.expr().visitTest(e));
		return result;
	}

	@Override
	public List<Expression> visitExprlist(
			ExprlistContext pctx) {
		if (!pctx.star_expr().isEmpty())
			throw new UnsupportedStatementException();

		List<Expression> result = new ArrayList<>(pctx.expr().size());
		if (pctx.expr().size() == 0)
			return result;
		for (ExprContext e : pctx.expr())
			result.add(ctx.expr().visitExpr(e));
		return result;
	}

	@Override
	public Statement visitImport_stmt(
			Import_stmtContext pctx) {
		if (pctx.import_from() != null)
			return visitImport_from(pctx.import_from());
		else
			return visitImport_name(pctx.import_name());
	}

	@Override
	public Statement visitImport_name(
			Import_nameContext pctx) {
		Map<String, String> libs = new HashMap<>();
		for (Dotted_as_nameContext single : pctx.dotted_as_names().dotted_as_name()) {
			String importedLibrary = dottedNameToString(single.dotted_name());
			String as = single.NAME() != null ? single.NAME().getSymbol().getText() : null;
			libs.put(importedLibrary, as);
		}

		Map.Entry<String, String> binding = libs.entrySet().stream().findFirst().orElse(null);
		if (binding != null) {
			String boundName = binding.getValue() == null ? binding.getKey() : binding.getValue();
			ctx.declareNameInCurrentScope(boundName);
		}

		String moduleName = libs.keySet().stream().findFirst().get();
		ModuleUnit module = ctx.importManager().importModule(moduleName);
		return new ImportModule(ctx.currentCFG(), support.getLocation(pctx), moduleName, module);
	}

	@Override
	public Statement visitImport_from(
			Import_fromContext pctx) {
		int dotCount = pctx.DOT().size() + pctx.ELLIPSIS().size() * 3;
		String name;
		if (dotCount == 0) {
			name = pctx.dotted_name() != null ? dottedNameToString(pctx.dotted_name()) : ".";
		} else {
			name = resolveRelativeImport(dotCount,
					pctx.dotted_name() != null ? dottedNameToString(pctx.dotted_name()) : null);
		}

		if (pctx.import_as_names() == null) {
			LibrarySpecificationProvider.importLibrary(ctx.program(), name, ctx.init());
			PyCFG pyCFG = new PyCFG(
					new CodeMemberDescriptor(support.getLocation(pctx), ctx.currentUnit(), false, "__init__"));
			pyCFG.addNode(new NoOp(pyCFG, SyntheticLocation.INSTANCE));
			return new ImportModule(ctx.currentCFG(), support.getLocation(pctx), name,
					(ModuleUnit) PyModuleType.lookup(name).getUnit());
		}
		Map<String, String> components = new LinkedHashMap<>();
		for (Import_as_nameContext single : pctx.import_as_names().import_as_name()) {
			String importedComponent = single.NAME(0).getSymbol().getText();
			String alias = single.NAME().size() == 2 ? single.NAME(1).getSymbol().getText() : importedComponent;
			components.put(importedComponent, alias);
			ctx.imports().put(alias, name + "." + importedComponent);
			ctx.declareNameInCurrentScope(alias);
		}
		ctx.importManager().importModule(name);

		for (Map.Entry<String, String> entry : components.entrySet()) {
			String qualifiedName = name + "." + entry.getKey();
			if (ctx.importManager().canResolveModule(qualifiedName))
				ctx.importManager().importModule(qualifiedName);
		}

		ModuleUnit currentModuleUnit = (ctx.currentUnit() instanceof ModuleUnit pmu) ? pmu : null;
		List<Pair<String, String>> importPairs = components.entrySet().stream()
				.map(e -> Pair.of(e.getValue(), e.getKey()))
				.collect(Collectors.toList());
		return new FromImport(ctx.currentCFG(), support.getLocation(pctx), currentModuleUnit, name, importPairs);
	}

	@Override
	public Object visitImport_as_name(
			Import_as_nameContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDotted_as_name(
			Dotted_as_nameContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitImport_as_names(
			Import_as_namesContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitDotted_as_names(
			Dotted_as_namesContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitDotted_name(
			Dotted_nameContext pctx) {
		if (pctx.NAME().isEmpty()) {
			return support.unsupported(pctx, "At least one name expected in Dotted_nameContext");
		}
		Expression result = null;
		String targetName = null;
		for (int i = 0; i < pctx.NAME().size(); i++) {
			TerminalNode name = pctx.NAME(i);
			if (targetName != null) {
				result = new AttributeAccess(ctx.currentCFG(), support.getLocation(pctx), result, name.getText());
			} else {
				result = support.makeRef(name.getText(), support.getLocation(pctx));
			}
			targetName = name.getText();
		}
		return result;
	}

	// === helpers used by this class and other sibling visitors ===

	public String dottedNameToString(
			Dotted_nameContext dotted_name) {
		return dotted_name.NAME().stream().map(t -> t.getSymbol().getText())
				.collect(Collectors.joining("."));
	}

	private String resolveRelativeImport(
			int dotCount,
			String dottedName) {
		String moduleName = (ctx.currentModule() != null) ? ctx.currentModule().getName() : "__main__";
		String packageName;
		if (ctx.currentFileIsPackage()) {
			packageName = moduleName;
		} else {
			int lastDot = moduleName.lastIndexOf('.');
			packageName = (lastDot >= 0) ? moduleName.substring(0, lastDot) : "";
		}
		// Entry-file fallback: the top-level main file carries the hardcoded
		// module name "__main__" and therefore has no package prefix to pull
		// from. When it lives inside a package (baseDir has __init__.py), the
		// import manager has already detected the dotted package name from the
		// directory chain — use it as the anchor, matching Python's
		// __package__ behavior for `python -m <pkg>.<mod>` execution.
		if (packageName.isEmpty()) {
			String detected = ctx.importManager().getPackageName();
			if (detected != null)
				packageName = detected;
		}
		String anchor = packageName;
		for (int i = 1; i < dotCount; i++) {
			int lastDot = anchor.lastIndexOf('.');
			anchor = (lastDot >= 0) ? anchor.substring(0, lastDot) : "";
		}
		if (dottedName == null || dottedName.isEmpty())
			return anchor;
		return anchor.isEmpty() ? dottedName : anchor + "." + dottedName;
	}

	public void declareAssignedNames(
			Expression target) {
		if (target instanceof VariableRef var) {
			ctx.declareNameInCurrentScope(var.getName());
			return;
		}

		if (target instanceof TupleCreation tuple)
			for (Expression sub : tuple.getSubExpressions())
				declareAssignedNames(sub);
		else if (target instanceof ListCreation list)
			for (Expression sub : list.getSubExpressions())
				declareAssignedNames(sub);
	}

	public Expression scopeAssignmentTarget(
			Expression target) {
		if (target instanceof AttributeAccess attr) {
			Expression receiver = attr.getSubExpression();
			if (receiver instanceof VariableRef var) {
				if (ctx.isNameInVisibleLocalScope(var.getName()))
					return new PyAccessInstanceGlobal(ctx.currentCFG(), attr.getLocation(), receiver,
							attr.getTarget());
				String receiverName = var.getName();
				String fqName = ctx.imports().getOrDefault(receiverName,
						ctx.currentModule() != null ? ctx.currentModule().getName() + "." + receiverName
								: null);
				if (fqName != null) {
					for (Unit u : ctx.program().getUnits()) {
						if (u instanceof CompilationUnit cu && cu.getName().equals(fqName))
							return support.makeScopedAttributeRef(cu, attr.getTarget(), attr.getLocation());
					}
				}
				// Receiver is not a known class — treat as module-level
				// instance variable. Wrap it in a scoped ref so the heap
				// domain can resolve it to the correct heap location.
				if (ctx.currentModule() != null) {
					Expression scopedReceiver = support.makeScopedAttributeRef(ctx.currentModule(), receiverName,
							var.getLocation());
					return new PyAccessInstanceGlobal(ctx.currentCFG(), attr.getLocation(), scopedReceiver,
							attr.getTarget());
				}
			}
			if (receiver instanceof PythonScopedAttributeAccessRef scopedRef) {
				String receiverName = scopedRef.getTarget().getName();
				String fqName = ctx.imports().getOrDefault(receiverName,
						ctx.currentModule() != null ? ctx.currentModule().getName() + "." + receiverName
								: null);
				if (fqName != null) {
					for (Unit u : ctx.program().getUnits()) {
						if (u instanceof CompilationUnit cu && cu.getName().equals(fqName))
							return support.makeScopedAttributeRef(cu, attr.getTarget(), attr.getLocation());
					}
				}
				return new PyAccessInstanceGlobal(ctx.currentCFG(), attr.getLocation(), receiver,
						attr.getTarget());
			}
			return target;
		}

		if (target instanceof VariableRef var) {
			if (ctx.isInsideLocalScope() && !(ctx.currentUnit() instanceof ClassUnit))
				return var;
			if (ctx.currentUnit() instanceof CompilationUnit cu)
				return support.makeScopedAttributeRef(cu, var.getName(), var.getLocation());
			if (ctx.currentModule() != null)
				return support.makeScopedAttributeRef(ctx.currentModule(), var.getName(), var.getLocation());
			return var;
		}

		if (target instanceof TupleCreation tuple) {
			Expression[] scopedSubs = new Expression[tuple.getSubExpressions().length];
			for (int i = 0; i < scopedSubs.length; i++)
				scopedSubs[i] = scopeAssignmentTarget(tuple.getSubExpressions()[i]);
			return new TupleCreation(tuple.getCFG(), tuple.getLocation(), scopedSubs);
		}

		if (target instanceof ListCreation list) {
			Expression[] scopedSubs = new Expression[list.getSubExpressions().length];
			for (int i = 0; i < scopedSubs.length; i++)
				scopedSubs[i] = scopeAssignmentTarget(list.getSubExpressions()[i]);
			return new ListCreation(list.getCFG(), list.getLocation(), scopedSubs);
		}

		return target;
	}
}
