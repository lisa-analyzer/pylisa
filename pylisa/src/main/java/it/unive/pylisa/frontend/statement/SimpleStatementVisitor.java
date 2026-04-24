package it.unive.pylisa.frontend.statement;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.AnnassignContext;
import it.unive.pylisa.antlr.Python3Parser.Assert_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.AugassignContext;
import it.unive.pylisa.antlr.Python3Parser.Del_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Expr_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Global_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Nonlocal_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Pass_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Simple_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.Small_stmtContext;
import it.unive.pylisa.antlr.Python3Parser.StmtContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_star_exprContext;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.PyAccessInstanceGlobal;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.frontend.expression.DunderMethods;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Triple;

/**
 * Handles Python simple statements — expression statements, del, pass, assert,
 * global/nonlocal, and the {@code small_stmt}/{@code simple_stmt} wrappers.
 * Extracted from {@link StatementVisitor} in Chunk 4. Cross-visitor recursion
 * routes through {@link ParserContext#stmt()} and {@link ParserContext#expr()}.
 */
public final class SimpleStatementVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public SimpleStatementVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Object visitStmt(
			StmtContext pctx) {
		if (pctx.simple_stmt() != null)
			return visitSimple_stmt(pctx.simple_stmt());
		else
			return ctx.stmt().visitCompound_stmt(pctx.compound_stmt());
	}

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

	public Statement visitSmall_stmt(
			Small_stmtContext pctx) {
		if (pctx.expr_stmt() != null)
			return visitExpr_stmt(pctx.expr_stmt());
		else if (pctx.del_stmt() != null)
			return visitDel_stmt(pctx.del_stmt());
		else if (pctx.pass_stmt() != null)
			return visitPass_stmt(pctx.pass_stmt());
		else if (pctx.import_stmt() != null)
			return ctx.stmt().visitImport_stmt(pctx.import_stmt());
		else if (pctx.assert_stmt() != null)
			return visitAssert_stmt(pctx.assert_stmt());
		else if (pctx.flow_stmt() != null)
			return ctx.stmt().visitFlow_stmt(pctx.flow_stmt());
		else if (pctx.nonlocal_stmt() != null)
			return new NoOp(ctx.currentCFG(), support.getLocation(pctx)); // TODO
		else if (pctx.global_stmt() != null)
			return new NoOp(ctx.currentCFG(), support.getLocation(pctx)); // TODO
		throw new UnsupportedStatementException("Simple statement not yet supported");
	}

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
				&& DunderMethods.GETITEM.equals(aa.getTarget())) {
			Expression receiver = fa.getSubExpressions()[1];
			Expression key = fa.getSubExpressions()[2];
			Expression rhs = visitTestlist_star_expr(pctx.testlist_star_expr(1));
			Expression setitemAttr = new AttributeAccess(
					ctx.currentCFG(), support.getLocation(pctx), receiver, DunderMethods.SETITEM);
			return new FunctionApply(ctx.currentCFG(), support.getLocation(pctx), setitemAttr,
					new Expression[] { receiver, key, rhs }, true);
		}

		return new PyAssign(ctx.currentCFG(), support.getLocation(pctx),
				target,
				visitTestlist_star_expr(pctx.testlist_star_expr(1)));
	}

	public Expression visitTestlist_star_expr(
			Testlist_star_exprContext pctx) {
		if (pctx.test().size() == 1)
			return ctx.expr().visitTest(pctx.test(0));

		support.unsound(pctx, "tuple creation treated as first element");
		return ctx.expr().visitTest(pctx.test(0));
	}

	public Object visitAnnassign(
			AnnassignContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitAugassign(
			AugassignContext pctx) {
		throw new UnsupportedStatementException();
	}

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
				ctx.stmt().visitExprlist(pctx.exprlist()).toArray(new Expression[pctx.exprlist().expr().size()]));
	}

	public Statement visitPass_stmt(
			Pass_stmtContext pctx) {
		return new NoOp(ctx.currentCFG(), support.getLocation(pctx));
	}

	public Object visitGlobal_stmt(
			Global_stmtContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitNonlocal_stmt(
			Nonlocal_stmtContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Expression visitAssert_stmt(
			Assert_stmtContext pctx) {
		return new UnresolvedCall(
				ctx.currentCFG(),
				support.getLocation(pctx),
				CallType.STATIC,
				"assert",
				Program.PROGRAM_NAME,
				LeftToRightEvaluation.INSTANCE,
				ctx.stmt().visitTestlist(pctx.testlist()).toArray(new Expression[pctx.testlist().test().size()]));
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
