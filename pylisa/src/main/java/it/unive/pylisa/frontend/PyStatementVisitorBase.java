package it.unive.pylisa.frontend;

import it.unive.lisa.program.*;
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
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.*;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.ModuleUnit;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.antlr.v4.runtime.tree.TerminalNode;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

public abstract class PyStatementVisitorBase extends PyExpressionVisitorBase {

	public PyStatementVisitorBase(
			String filePath,
			boolean notebook) {
		super(filePath, notebook);
	}

	public PyStatementVisitorBase(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	public PyStatementVisitorBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	public PyStatementVisitorBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder,
			String sourceRoot) {
		super(filePath, notebook, cellOrder, sourceRoot);
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
			if (last != null && !last.stopsExecution())
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
			return new NoOp(this.ctx.currentCFG(), getLocation(ctx)); // TODO
		else if (ctx.global_stmt() != null)
			return new NoOp(this.ctx.currentCFG(), getLocation(ctx)); // TODO
		throw new UnsupportedStatementException("Simple statement not yet supported");
	}

	@Override
	public Expression visitExpr_stmt(
			Expr_stmtContext ctx) {
		if (ctx.annassign() != null) {
			boolean oldPrepend = this.ctx.shouldPrependUnitAccess();
			this.ctx.shouldPrependUnitAccess(false);
			Expression rawTarget = visitTestlist_star_expr(ctx.testlist_star_expr(0));
			this.ctx.shouldPrependUnitAccess(oldPrepend);

			declareAssignedNames(rawTarget);
			Expression target = scopeAssignmentTarget(rawTarget);
			if (ctx.annassign().ASSIGN() != null && ctx.annassign().test().size() > 1)
				return new PyAssign(this.ctx.currentCFG(), getLocation(ctx), target,
						visitTest(ctx.annassign().test(1)));

			// Type-only annotations do not carry runtime effects in our current
			// model.
			return target;
		}

		if (ctx.ASSIGN().size() == 0)
			if (ctx.testlist_star_expr().size() != 1)
				// augassign or annassign have been used, both not supported
				throw new UnsupportedStatementException();
			else
				return visitTestlist_star_expr(ctx.testlist_star_expr(0));

		boolean oldPrepend = this.ctx.shouldPrependUnitAccess();
		this.ctx.shouldPrependUnitAccess(false);
		Expression rawTarget = visitTestlist_star_expr(ctx.testlist_star_expr(0));
		this.ctx.shouldPrependUnitAccess(oldPrepend);

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
			Expression rhs = visitTestlist_star_expr(ctx.testlist_star_expr(1));
			Expression setitemAttr = new AttributeAccess(
					this.ctx.currentCFG(), getLocation(ctx), receiver, "__setitem__");
			return new FunctionApply(this.ctx.currentCFG(), getLocation(ctx), setitemAttr,
					new Expression[] { receiver, key, rhs }, true);
		}

		PyAssign assign = new PyAssign(this.ctx.currentCFG(), getLocation(ctx),
				target,
				visitTestlist_star_expr(ctx.testlist_star_expr(1)));
		return assign;
	}

	@Override
	public Expression visitTestlist_star_expr(
			Testlist_star_exprContext ctx) {
		if (ctx.test().size() == 1)
			return visitTest(ctx.test(0));

		unsound(ctx, "tuple creation treated as first element");
		return visitTest(ctx.test(0));
	}

	@Override
	public Object visitAnnassign(
			AnnassignContext ctx) {
		throw new UnsupportedStatementException();
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
			return unsupported(ctx, "We support only expressions without * in del statements");
		Statement result = new UnresolvedCall(
				this.ctx.currentCFG(),
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
		return new NoOp(this.ctx.currentCFG(), getLocation(ctx));
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
				this.ctx.currentCFG(),
				getLocation(ctx),
				CallType.STATIC,
				"assert",
				Program.PROGRAM_NAME,
				LeftToRightEvaluation.INSTANCE,
				visitTestlist(ctx.testlist())
						.toArray(new Expression[ctx.testlist().test().size()]));
	}

	@Override
	public Statement visitFlow_stmt(
			Flow_stmtContext ctx) {
		if (ctx.return_stmt() != null)
			return visitReturn_stmt(ctx.return_stmt());

		if (ctx.raise_stmt() != null) {
			unsound(ctx, "raise treated as no-op");
			return new NoOp(this.ctx.currentCFG(), getLocation(ctx));
		}

		if (ctx.yield_stmt() != null) {
			Yield_argContext yieldArg = ctx.yield_stmt().yield_expr().yield_arg();
			if (yieldArg == null) {
				// bare 'yield' — treat as a no-op (control flow continues to
				// cleanup code)
				return new NoOp(this.ctx.currentCFG(), getLocation(ctx));
			}
			List<Expression> l = extractExpressionsFromYieldArg(yieldArg);
			return new UnresolvedCall(
					this.ctx.currentCFG(),
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
	public Statement visitReturn_stmt(
			Return_stmtContext ctx) {
		if (ctx.testlist() == null)
			return new Ret(this.ctx.currentCFG(), getLocation(ctx));
		if (ctx.testlist().test().size() == 1)
			return new Return(this.ctx.currentCFG(), getLocation(ctx), visitTest(ctx.testlist().test(0)));
		else {
			unsound(ctx, "multiple return values treated as first value");
			return new Return(this.ctx.currentCFG(), getLocation(ctx), visitTest(ctx.testlist().test(0)));
		}
	}

	@Override
	public Statement visitBreak_stmt(
			Break_stmtContext ctx) {
		return new Break(this.ctx.currentCFG(), getLocation(ctx));
	}

	@Override
	public Statement visitContinue_stmt(
			Continue_stmtContext ctx) {
		return new Continue(this.ctx.currentCFG(), getLocation(ctx));
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
		else if (ctx.classdef() != null)
			return this.visitClassdef(ctx.classdef());
		else if (ctx.decorated() != null)
			return this.visitDecorated(ctx.decorated());
		else if (ctx.async_stmt() != null)
			return this.visitAsync_stmt(ctx.async_stmt());
		return unsupported(ctx, "Statement not yet supported");
	}

	@Override
	public Object visitAsync_stmt(
			Async_stmtContext ctx) {
		unsound(ctx, "async stmt treated as sync");
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
		NoOp ifExitNode = new NoOp(this.ctx.currentCFG(), getLocation(ctx));
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
		int testLength = ctx.test().size();
		Statement lastElifGuard = booleanGuard;
		if (testLength > 1)
			// if testLength is >1 the context contains elif
			for (int i = 1; i < testLength; i++) {
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
			this.ctx.cfs().add(new IfThenElse(this.ctx.currentCFG().getNodeList(), branch.getLeft(), ifExitNode,
					branch.getRight(),
					new HashSet<>(falseStatements)));
		}
		this.ctx.cfs().add(new IfThenElse(this.ctx.currentCFG().getNodeList(), booleanGuard, ifExitNode,
				trueBlock.getMiddle().getNodes(),
				falseStatements));
		return Triple.of(booleanGuard, block, ifExitNode);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWhile_stmt(
			While_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of while
		NoOp whileExitNode = new NoOp(this.ctx.currentCFG(), getLocation(ctx));
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
		if (!trueBlock.getRight().stopsExecution())
			block.addEdge(new SequentialEdge(trueBlock.getRight(), condition));

		// check if there's an else condition for the while
		Statement firstFollower;
		if (ctx.ELSE() != null) {
			Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> falseBlock = visitSuite(ctx.suite(1));
			block.mergeWith(falseBlock.getMiddle());
			block.addEdge(new FalseEdge(condition, falseBlock.getLeft()));
			if (!falseBlock.getRight().stopsExecution())
				block.addEdge(new SequentialEdge(falseBlock.getRight(), whileExitNode));
			firstFollower = falseBlock.getLeft();
		} else {
			block.addEdge(new FalseEdge(condition, whileExitNode));
			firstFollower = whileExitNode;
		}

		this.ctx.cfs().add(new Loop(this.ctx.currentCFG().getNodeList(), condition, firstFollower,
				trueBlock.getMiddle().getNodes()));
		return Triple.of(condition, block, whileExitNode);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFor_stmt(
			For_stmtContext ctx) {
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		// create and add exit point of for
		NoOp exit = new NoOp(this.ctx.currentCFG(), getLocation(ctx));
		block.addNode(exit);

		List<Expression> exprs = visitExprlist(ctx.exprlist());
		Expression variable;
		if (exprs.size() == 1)
			variable = exprs.get(0);
		else
			variable = new TupleCreation(this.ctx.currentCFG(), getLocation(ctx), exprs.toArray(Expression[]::new));

		List<Expression> list = visitTestlist(ctx.testlist());
		Expression collection = list.iterator().next();

		VariableRef counter = new VariableRef(
				this.ctx.currentCFG(),
				getLocation(ctx),
				"__counter_location" + getLocation(ctx).getLine(), Int32Type.INSTANCE);
		Expression[] counter_pars = { collection, counter };

		// counter = 0;
		Assignment counter_init = new Assignment(
				this.ctx.currentCFG(),
				getLocation(ctx),
				counter,
				new Int32Literal(this.ctx.currentCFG(), getLocation(ctx), 0));
		block.addNode(counter_init);

		// counter < collection.size()
		UnresolvedCall condition = new UnresolvedCall(
				this.ctx.currentCFG(),
				getLocation(ctx),
				CallType.INSTANCE,
				null,
				"__lt__",
				counter,
				new UnresolvedCall(
						this.ctx.currentCFG(),
						getLocation(ctx),
						CallType.INSTANCE,
						null,
						"__len__",
						LeftToRightEvaluation.INSTANCE,
						collection));
		block.addNode(condition);

		// element = collection.at(counter)
		Assignment element_assignment = new Assignment(
				this.ctx.currentCFG(),
				getLocation(ctx),
				variable,
				new UnresolvedCall(
						this.ctx.currentCFG(),
						getLocation(ctx),
						CallType.INSTANCE,
						null,
						"__getitem__",
						LeftToRightEvaluation.INSTANCE,
						counter_pars));
		block.addNode(element_assignment);

		// counter = counter + 1;
		Assignment counter_increment = new Assignment(
				this.ctx.currentCFG(),
				getLocation(ctx),
				counter,
				new PyAddition(
						this.ctx.currentCFG(),
						getLocation(ctx),
						counter,
						new Int32Literal(
								this.ctx.currentCFG(),
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
		if (!body.getRight().stopsExecution())
			block.addEdge(new SequentialEdge(body.getRight(), counter_increment));
		block.addEdge(new SequentialEdge(counter_increment, condition));
		block.addEdge(new FalseEdge(condition, exit));

		Collection<Statement> nodes = new HashSet<>(body.getMiddle().getNodes());
		nodes.add(element_assignment);
		nodes.add(counter_increment);
		this.ctx.cfs().add(new Loop(this.ctx.currentCFG().getNodeList(), condition, exit, nodes));
		return Triple.of(counter_init, block, exit);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitTry_stmt(
			Try_stmtContext ctx) {
		unsound(ctx, "try block: except clauses conservatively modeled as a bypass");
		// Model `try: BODY except: …` as "BODY may execute fully OR an
		// exception may be caught anywhere inside, in which case control
		// resumes at the try-exit carrying the pre-try state". Structurally:
		//
		// entry(NoOp) ──► BODY.entry ──► … ──► BODY.exit ──► exit(NoOp)
		// └─────────────────────────────────────────────► exit
		//
		// The parallel edge from entry to exit gives the fixpoint a path that
		// preserves the pre-try state, so if the body produces bottom (e.g.
		// because an unresolvable `from X import Y` cascaded), the exit still
		// receives at least the entry state by lattice join — matching the
		// semantics of Python's bare-`except` over cross-import try/blocks
		// common in package `__init__.py` files.
		Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> body = visitSuite(ctx.suite(0));
		NodeList<CFG, Statement, Edge> block = new NodeList<>(SEQUENTIAL_SINGLETON);
		NoOp entry = new NoOp(this.ctx.currentCFG(), getLocation(ctx));
		NoOp exit = new NoOp(this.ctx.currentCFG(), getLocation(ctx));
		block.addNode(entry);
		block.addNode(exit);
		block.mergeWith(body.getMiddle());
		block.addEdge(new SequentialEdge(entry, body.getLeft()));
		// Only connect the body's natural exit to our wrapper exit when the
		// body's last node can fall through — return/raise/break/continue are
		// execution-stopping and must not have outgoing edges (CFG.validate
		// would reject it).
		if (body.getRight() != null && !body.getRight().stopsExecution())
			block.addEdge(new SequentialEdge(body.getRight(), exit));
		// The bypass edge — control jumps straight to the exit as if every
		// statement in the body had raised an exception that was caught.
		block.addEdge(new SequentialEdge(entry, exit));
		return Triple.of(entry, block, exit);
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
					continue;

				@SuppressWarnings("unchecked")
				Triple<Statement, NodeList<CFG, Statement, Edge>,
						Statement> st = (Triple<Statement, NodeList<CFG, Statement, Edge>, Statement>) parsed;
				block.mergeWith(st.getMiddle());
				if (first == null)
					// this is the first instruction
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
	public Statement visitImport_stmt(
			Import_stmtContext ctx) {
		if (ctx.import_from() != null)
			return visitImport_from(ctx.import_from());
		else
			return visitImport_name(ctx.import_name());
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

		Map.Entry<String, String> binding = libs.entrySet().stream().findFirst().orElse(null);
		if (binding != null) {
			String boundName = binding.getValue() == null ? binding.getKey() : binding.getValue();
			declareNameInCurrentScope(boundName);
		}

		String moduleName = libs.keySet().stream().findFirst().get();
		ModuleUnit module = this.ctx.importManager().importModule(moduleName);
		return new ImportModule(this.ctx.currentCFG(), getLocation(ctx), moduleName, module);
	}

	@Override
	public Statement visitImport_from(
			Import_fromContext ctx) {
		int dotCount = ctx.DOT().size() + ctx.ELLIPSIS().size() * 3;
		String name;
		if (dotCount == 0) {
			name = ctx.dotted_name() != null ? dottedNameToString(ctx.dotted_name()) : ".";
		} else {
			name = resolveRelativeImport(dotCount,
					ctx.dotted_name() != null ? dottedNameToString(ctx.dotted_name()) : null);
		}

		if (ctx.import_as_names() == null) {
			LibrarySpecificationProvider.importLibrary(this.ctx.program(), name, init);
			PyCFG pyCFG = new PyCFG(
					new CodeMemberDescriptor(getLocation(ctx), this.ctx.currentUnit(), false, "__init__"));
			pyCFG.addNode(new NoOp(pyCFG, SyntheticLocation.INSTANCE));
			return new ImportModule(this.ctx.currentCFG(), getLocation(ctx), name,
					(ModuleUnit) PyModuleType.lookup(name).getUnit());
		}
		Map<String, String> components = new java.util.LinkedHashMap<>();
		for (Import_as_nameContext single : ctx.import_as_names().import_as_name()) {
			String importedComponent = single.NAME(0).getSymbol().getText();
			String alias = single.NAME().size() == 2 ? single.NAME(1).getSymbol().getText() : importedComponent;
			components.put(importedComponent, alias);
			this.ctx.imports().put(alias, name + "." + importedComponent);
			declareNameInCurrentScope(alias);
		}
		this.ctx.importManager().importModule(name);

		for (Map.Entry<String, String> entry : components.entrySet()) {
			String qualifiedName = name + "." + entry.getKey();
			if (this.ctx.importManager().canResolveModule(qualifiedName))
				this.ctx.importManager().importModule(qualifiedName);
		}

		ModuleUnit currentModuleUnit = (this.ctx.currentUnit() instanceof ModuleUnit pmu) ? pmu : null;
		List<Pair<String, String>> importPairs = components.entrySet().stream()
				.map(e -> Pair.of(e.getValue(), e.getKey()))
				.collect(java.util.stream.Collectors.toList());
		return new FromImport(this.ctx.currentCFG(), getLocation(ctx), currentModuleUnit, name, importPairs);
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
	public Expression visitDotted_name(
			Dotted_nameContext ctx) {
		if (ctx.NAME().isEmpty()) {
			return unsupported(ctx, "At least one name expected in Dotted_nameContext");
		}
		Expression result = null;
		String targetName = null;
		for (int i = 0; i < ctx.NAME().size(); i++) {
			TerminalNode name = ctx.NAME(i);
			if (targetName != null) {
				result = new AttributeAccess(this.ctx.currentCFG(), getLocation(ctx), result, name.getText());
			} else {
				result = makeRef(name.getText(), getLocation(ctx));
			}
			targetName = name.getText();
		}
		return result;
	}

	private String resolveRelativeImport(
			int dotCount,
			String dottedName) {
		String moduleName = (this.ctx.currentModule() != null) ? this.ctx.currentModule().getName() : "__main__";
		String packageName;
		if (currentFileIsPackage) {
			packageName = moduleName;
		} else {
			int lastDot = moduleName.lastIndexOf('.');
			packageName = (lastDot >= 0) ? moduleName.substring(0, lastDot) : "";
		}
		// Entry-file fallback: the top-level main file carries the hardcoded
		// module name "__main__" and therefore has no package prefix to pull
		// from. When it lives inside a package (baseDir has __init__.py), the
		// import manager has already detected the dotted package name from
		// the directory chain — use it as the anchor, matching Python's
		// __package__ behavior for `python -m <pkg>.<mod>` execution.
		if (packageName.isEmpty()) {
			String detected = this.ctx.importManager().getPackageName();
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

	protected String dottedNameToString(
			Dotted_nameContext dotted_name) {
		return dotted_name.NAME().stream().map(t -> t.getSymbol().getText())
				.collect(java.util.stream.Collectors.joining("."));
	}

	private void declareAssignedNames(
			Expression target) {
		if (target instanceof VariableRef var) {
			declareNameInCurrentScope(var.getName());
			return;
		}

		if (target instanceof TupleCreation tuple)
			for (Expression sub : tuple.getSubExpressions())
				declareAssignedNames(sub);
		else if (target instanceof ListCreation list)
			for (Expression sub : list.getSubExpressions())
				declareAssignedNames(sub);
	}

	private Expression scopeAssignmentTarget(
			Expression target) {
		if (target instanceof AttributeAccess attr) {
			Expression receiver = attr.getSubExpression();
			if (receiver instanceof VariableRef var) {
				if (isNameInVisibleLocalScope(var.getName()))
					return new PyAccessInstanceGlobal(this.ctx.currentCFG(), attr.getLocation(), receiver,
							attr.getTarget());
				// Try to resolve receiver as a CompilationUnit (e.g. Class.Y =
				// 100)
				String receiverName = var.getName();
				String fqName = this.ctx.imports().getOrDefault(receiverName,
						this.ctx.currentModule() != null ? this.ctx.currentModule().getName() + "." + receiverName
								: null);
				if (fqName != null) {
					for (Unit u : this.ctx.program().getUnits()) {
						if (u instanceof CompilationUnit cu && cu.getName().equals(fqName))
							return makeScopedAttributeRef(cu, attr.getTarget(), attr.getLocation());
					}
				}
				// Receiver is not a known class — treat as module-level
				// instance
				// variable. Wrap it in a scoped ref so the heap domain can
				// resolve it to the correct heap location.
				if (this.ctx.currentModule() != null) {
					Expression scopedReceiver = makeScopedAttributeRef(this.ctx.currentModule(), receiverName,
							var.getLocation());
					return new PyAccessInstanceGlobal(this.ctx.currentCFG(), attr.getLocation(), scopedReceiver,
							attr.getTarget());
				}
			}
			if (receiver instanceof PythonScopedAttributeAccessRef scopedRef) {
				String receiverName = scopedRef.getTarget().getName();
				String fqName = this.ctx.imports().getOrDefault(receiverName,
						this.ctx.currentModule() != null ? this.ctx.currentModule().getName() + "." + receiverName
								: null);
				if (fqName != null) {
					for (Unit u : this.ctx.program().getUnits()) {
						if (u instanceof CompilationUnit cu && cu.getName().equals(fqName))
							return makeScopedAttributeRef(cu, attr.getTarget(), attr.getLocation());
					}
				}
				// Receiver is an instance variable -> instance attribute heap
				// write
				return new PyAccessInstanceGlobal(this.ctx.currentCFG(), attr.getLocation(), receiver,
						attr.getTarget());
			}
			return target;
		}

		if (target instanceof VariableRef var) {
			if (isInsideLocalScope() && !(this.ctx.currentUnit() instanceof ClassUnit))
				return var;
			if (this.ctx.currentUnit() instanceof CompilationUnit cu)
				return makeScopedAttributeRef(cu, var.getName(), var.getLocation());
			if (this.ctx.currentModule() != null)
				return makeScopedAttributeRef(this.ctx.currentModule(), var.getName(), var.getLocation());
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
