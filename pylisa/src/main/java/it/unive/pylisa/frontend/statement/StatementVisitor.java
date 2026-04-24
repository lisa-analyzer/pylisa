package it.unive.pylisa.frontend.statement;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
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
import it.unive.pylisa.antlr.Python3Parser.Yield_stmtContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Triple;

/**
 * Root dispatcher for Python statement parsing. Delegates each grammar rule to
 * one of four category sub-visitors introduced in Chunk 4:
 * {@link SimpleStatementVisitor}, {@link ControlFlowVisitor},
 * {@link FlowControlVisitor}, and {@link ImportVisitor}. Cross-category
 * recursion inside those sub-visitors routes back through
 * {@link ParserContext#stmt()}, so every rule resolves via this dispatcher.
 */
public final class StatementVisitor extends Python3ParserBaseVisitor<Object> {

	private final ParserContext ctx;

	private final SimpleStatementVisitor simple;
	private final ControlFlowVisitor control;
	private final FlowControlVisitor flow;
	private final ImportVisitor imports;

	public StatementVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		Objects.requireNonNull(support);
		this.simple = new SimpleStatementVisitor(ctx, support);
		this.control = new ControlFlowVisitor(ctx, support);
		this.flow = new FlowControlVisitor(ctx, support);
		this.imports = new ImportVisitor(ctx, support);
	}

	// === control flow ===

	public PyCFG visitFile_input(
			File_inputContext c) {
		return control.visitFile_input(c);
	}

	@Override
	public Object visitCompound_stmt(
			Compound_stmtContext c) {
		return control.visitCompound_stmt(c);
	}

	@Override
	public Object visitAsync_stmt(
			Async_stmtContext c) {
		return control.visitAsync_stmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitIf_stmt(
			If_stmtContext c) {
		return control.visitIf_stmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWhile_stmt(
			While_stmtContext c) {
		return control.visitWhile_stmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitFor_stmt(
			For_stmtContext c) {
		return control.visitFor_stmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitTry_stmt(
			Try_stmtContext c) {
		return control.visitTry_stmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_stmt(
			With_stmtContext c) {
		return control.visitWith_stmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitWith_item(
			With_itemContext c) {
		return control.visitWith_item(c);
	}

	@Override
	public Object visitExcept_clause(
			Except_clauseContext c) {
		return control.visitExcept_clause(c);
	}

	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSuite(
			SuiteContext c) {
		return control.visitSuite(c);
	}

	// === simple statements ===

	@Override
	public Object visitStmt(
			StmtContext c) {
		return simple.visitStmt(c);
	}

	@Override
	public Triple<Statement, NodeList<CFG, Statement, Edge>, Statement> visitSimple_stmt(
			Simple_stmtContext c) {
		return simple.visitSimple_stmt(c);
	}

	@Override
	public Statement visitSmall_stmt(
			Small_stmtContext c) {
		return simple.visitSmall_stmt(c);
	}

	@Override
	public Expression visitExpr_stmt(
			Expr_stmtContext c) {
		return simple.visitExpr_stmt(c);
	}

	@Override
	public Expression visitTestlist_star_expr(
			Testlist_star_exprContext c) {
		return simple.visitTestlist_star_expr(c);
	}

	@Override
	public Object visitAnnassign(
			AnnassignContext c) {
		return simple.visitAnnassign(c);
	}

	@Override
	public Object visitAugassign(
			AugassignContext c) {
		return simple.visitAugassign(c);
	}

	@Override
	public Statement visitDel_stmt(
			Del_stmtContext c) {
		return simple.visitDel_stmt(c);
	}

	@Override
	public Statement visitPass_stmt(
			Pass_stmtContext c) {
		return simple.visitPass_stmt(c);
	}

	@Override
	public Object visitGlobal_stmt(
			Global_stmtContext c) {
		return simple.visitGlobal_stmt(c);
	}

	@Override
	public Object visitNonlocal_stmt(
			Nonlocal_stmtContext c) {
		return simple.visitNonlocal_stmt(c);
	}

	@Override
	public Expression visitAssert_stmt(
			Assert_stmtContext c) {
		return simple.visitAssert_stmt(c);
	}

	// === flow control ===

	@Override
	public Statement visitFlow_stmt(
			Flow_stmtContext c) {
		return flow.visitFlow_stmt(c);
	}

	@Override
	public Statement visitReturn_stmt(
			Return_stmtContext c) {
		return flow.visitReturn_stmt(c);
	}

	@Override
	public Statement visitBreak_stmt(
			Break_stmtContext c) {
		return flow.visitBreak_stmt(c);
	}

	@Override
	public Statement visitContinue_stmt(
			Continue_stmtContext c) {
		return flow.visitContinue_stmt(c);
	}

	@Override
	public Object visitYield_stmt(
			Yield_stmtContext c) {
		return flow.visitYield_stmt(c);
	}

	@Override
	public Object visitRaise_stmt(
			Raise_stmtContext c) {
		return flow.visitRaise_stmt(c);
	}

	// === imports ===

	@Override
	public Statement visitImport_stmt(
			Import_stmtContext c) {
		return imports.visitImport_stmt(c);
	}

	@Override
	public Statement visitImport_name(
			Import_nameContext c) {
		return imports.visitImport_name(c);
	}

	@Override
	public Statement visitImport_from(
			Import_fromContext c) {
		return imports.visitImport_from(c);
	}

	@Override
	public Object visitImport_as_name(
			Import_as_nameContext c) {
		return imports.visitImport_as_name(c);
	}

	@Override
	public Object visitDotted_as_name(
			Dotted_as_nameContext c) {
		return imports.visitDotted_as_name(c);
	}

	@Override
	public Object visitImport_as_names(
			Import_as_namesContext c) {
		return imports.visitImport_as_names(c);
	}

	@Override
	public Object visitDotted_as_names(
			Dotted_as_namesContext c) {
		return imports.visitDotted_as_names(c);
	}

	@Override
	public Expression visitDotted_name(
			Dotted_nameContext c) {
		return imports.visitDotted_name(c);
	}

	// === expression-list helpers shared across categories ===

	public List<Expression> visitTestlist(
			TestlistContext pctx) {
		List<Expression> result = new ArrayList<>(pctx.test().size());
		if (pctx.test().size() == 0)
			return result;
		for (TestContext e : pctx.test())
			result.add(ctx.expr().visitTest(e));
		return result;
	}

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

}
