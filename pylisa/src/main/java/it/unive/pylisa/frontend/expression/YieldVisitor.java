package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.Encoding_declContext;
import it.unive.pylisa.antlr.Python3Parser.SliceopContext;
import it.unive.pylisa.antlr.Python3Parser.Star_exprContext;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.TrailerContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_exprContext;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Consolidates the currently-unsupported Python grammar productions (yield,
 * star expression, encoding declaration, raw trailer/subscriptlist/sliceop
 * dispatch) plus the one real helper {@code extractExpressionsFromYieldArg}.
 * Extracted from {@link ExpressionVisitor} in Chunk 3. Chunk 8 is expected to
 * collapse the stubs into a single registry.
 */
public final class YieldVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public YieldVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Object visitYield_expr(
			Yield_exprContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitYield_arg(
			Yield_argContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitStar_expr(
			Star_exprContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitEncoding_decl(
			Encoding_declContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitTrailer(
			TrailerContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitSubscriptlist(
			SubscriptlistContext pctx) {
		throw new UnsupportedStatementException();
	}

	public Object visitSliceop(
			SliceopContext pctx) {
		throw new UnsupportedStatementException();
	}

	public List<Expression> extractExpressionsFromYieldArg(
			Yield_argContext pctx) {
		if (pctx.test() != null) {
			List<Expression> r = new ArrayList<>(1);
			r.add(ctx.expr().visitTest(pctx.test()));
			return r;
		}
		return ctx.stmt().visitTestlist(pctx.testlist());
	}
}
