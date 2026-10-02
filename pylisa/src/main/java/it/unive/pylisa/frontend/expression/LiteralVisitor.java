package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Float64Literal;
import it.unive.lisa.program.cfg.statement.literal.Int64Literal;
import it.unive.lisa.program.type.Int64Type;
import it.unive.pylisa.cfg.expression.literal.PyUnknownLiteral;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.AtomContext;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
import it.unive.pylisa.antlr.Python3Parser.TestOrStarContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_compContext;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.SetCreation;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.expression.literal.PyEllipsisLiteral;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.antlr.v4.runtime.ParserRuleContext;
import org.apache.commons.lang3.tuple.Pair;

/**
 * Handles atom leaves: name references, numeric/string/bool/None/ellipsis
 * literals, and collection literals (list, tuple, set, dict). Extracted from
 * {@link ExpressionVisitor} in Chunk 3. Cross-visitor recursion (e.g. inside
 * collection literals) goes through {@code ctx.expr()}.
 */
public final class LiteralVisitor {

	private final ParserContext ctx;
	private final ParserSupport support;

	public LiteralVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Expression visitAtom(
			AtomContext pctx) {
		if (pctx.NAME() != null)
			return visitNameAtom(pctx);
		if (pctx.NUMBER() != null)
			return visitNumberAtom(pctx);
		if (pctx.FALSE() != null)
			return new FalseLiteral(ctx.currentCFG(), support.getLocation(pctx));
		if (pctx.TRUE() != null)
			return new TrueLiteral(ctx.currentCFG(), support.getLocation(pctx));
		if (pctx.NONE() != null)
			return new PyNoneLiteral(ctx.currentCFG(), support.getLocation(pctx));
		if (pctx.STRING().size() > 0) {
			if (pctx.STRING().size() > 1)
				support.limitation(pctx, "adjacent string literals (only the first is kept)");
			return support.strip(support.getLocation(pctx), pctx.STRING(0).getText());
		}
		if (pctx.yield_expr() != null)
			return support.rejectUnsupported(pctx, "yield expression in atom");
		if (pctx.OPEN_BRACE() == null && pctx.dictorsetmaker() != null)
			return visitDictorsetmaker(pctx.dictorsetmaker());
		if (pctx.OPEN_BRACK() != null)
			return visitListLiteral(pctx);
		if (pctx.OPEN_PAREN() != null)
			return visitTupleOrParenthesized(pctx);
		if (pctx.OPEN_BRACE() != null)
			return visitDictOrSetLiteral(pctx);
		if (pctx.ELLIPSIS() != null)
			return new PyEllipsisLiteral(ctx.currentCFG(), support.getLocation(pctx));
		return support.rejectUnsupported(pctx, "atom variant");
	}

	private Expression visitNameAtom(
			AtomContext pctx) {
		return support.makeRef(pctx.NAME().getText(), support.getLocation(pctx));
	}

	private Expression visitNumberAtom(
			AtomContext pctx) {
		PythonNumericLiteral.Parsed parsed = PythonNumericLiteral.parse(pctx.NUMBER().getText());
		return switch (parsed) {
		case PythonNumericLiteral.IntegerLit i -> new Int32Literal(ctx.currentCFG(),
				support.getLocation(pctx), i.value());
		case PythonNumericLiteral.LongLit l -> new Int64Literal(ctx.currentCFG(),
				support.getLocation(pctx), l.value());
		case PythonNumericLiteral.BigIntegerLit b -> new PyUnknownLiteral(ctx.currentCFG(),
				support.getLocation(pctx), pctx.NUMBER().getText(), Int64Type.INSTANCE);
		case PythonNumericLiteral.FloatLit f -> new Float64Literal(ctx.currentCFG(),
				support.getLocation(pctx), f.value());
		case PythonNumericLiteral.ComplexLit c -> support.rejectUnsupported(pctx, "complex literal");
		};
	}

	private Expression visitListLiteral(
			AtomContext pctx) {
		List<Expression> sts = extractExpressionsFromTestlist_comp(pctx.testlist_comp());
		return new ListCreation(ctx.currentCFG(), support.getLocation(pctx), sts.toArray(Expression[]::new));
	}

	private Expression visitTupleOrParenthesized(
			AtomContext pctx) {
		if (pctx.yield_expr() != null)
			return support.rejectUnsupported(pctx, "yield expression in parens");
		List<Expression> sts = extractExpressionsFromTestlist_comp(pctx.testlist_comp());
		if (sts.size() <= 1)
			return sts.isEmpty() ? new TupleCreation(ctx.currentCFG(), support.getLocation(pctx)) : sts.get(0);
		support.unsound(pctx, "tuple creation treated as first element");
		return sts.get(0);
	}

	private Expression visitDictOrSetLiteral(
			AtomContext pctx) {
		if (!support.isADict(pctx.dictorsetmaker())) {
			List<Expression> values = extractElementsFromSet(pctx.dictorsetmaker());
			return new SetCreation(ctx.currentCFG(), support.getLocation(pctx),
					values.toArray(Expression[]::new));
		}
		List<Pair<Expression, Expression>> values = extractPairsFromDict(pctx.dictorsetmaker());
		@SuppressWarnings("unchecked")
		DictionaryCreation r = new DictionaryCreation(ctx.currentCFG(), support.getLocation(pctx),
				values.toArray(Pair[]::new));
		return r;
	}

	public Expression visitDictorsetmaker(
			DictorsetmakerContext pctx) {
		if (pctx.COLON().size() == 0) {
			List<Expression> values = new ArrayList<>();
			for (TestContext exp : pctx.test())
				values.add(ctx.expr().visitTest(exp));
			return new SetCreation(ctx.currentCFG(), support.getLocation(pctx),
					values.toArray(Expression[]::new));
		}
		return support.rejectUnsupported(pctx, "dict comprehension");
	}

	public Expression visitTestlist_comp(
			Testlist_compContext pctx) {
		if (pctx.comp_for() != null)
			return support.rejectUnsupported(pctx, "generator comprehension");
		return visitTestlistFirstElement(pctx);
	}

	public Expression visitTestOrStar(
			TestOrStarContext pctx) {
		if (pctx.star_expr() != null)
			return visitStarExprUnsound(pctx.star_expr(), pctx);
		return ctx.expr().visitTest(pctx.test());
	}

	/**
	 * The first element of a {@code testlist_comp} is a {@code namedexpr_test}
	 * (or {@code star_expr}) to accommodate walrus inside parens like
	 * {@code (x := 5)}. All subsequent elements after a comma are plain
	 * {@code testOrStar} since PEP 572 forbids walrus as a tuple member.
	 */
	private Expression visitTestlistFirstElement(
			Testlist_compContext pctx) {
		if (pctx.namedexpr_test() != null)
			return ctx.expr().visitNamedexpr_test(pctx.namedexpr_test());
		if (pctx.star_expr() != null)
			return visitStarExprUnsound(pctx.star_expr(), pctx);
		return visitTestOrStar(pctx.testOrStar(0));
	}

	/**
	 * **Unsound translation** of a {@code star_expr} ({@code *expr}) used in
	 * tuple/list literal contexts (e.g. {@code [*xs, y]}, {@code (*head, tail)}).
	 * Python's iterable-unpacking semantics inline every element of the iterable
	 * into the surrounding collection. We don't model that here: we visit the
	 * inner expression and return it as-is, treating {@code *xs} as {@code xs}.
	 * Routing/network analysis is unaffected (the iterable's identity is what
	 * matters for handler resolution); precision is lost where the elements of
	 * the unpacked iterable would feed downstream value-domain reasoning.
	 * <p>
	 * Same shape as {@code DecoratorVisitor}'s class-decorator skip-and-log:
	 * record an {@code UNSOUND} diagnostic so the missing modeling is visible
	 * in the run output, then keep the analysis reachable.
	 */
	private Expression visitStarExprUnsound(
			it.unive.pylisa.antlr.Python3Parser.Star_exprContext starCtx,
			ParserRuleContext outerCtx) {
		support.unsound(outerCtx, "star expression in testOrStar (treating `*xs` as `xs` — iterable-unpack semantics dropped)");
		return ctx.expr().visitExpr(starCtx.expr());
	}

	public List<Expression> extractExpressionsFromTestlist_comp(
			Testlist_compContext pctx) {
		List<Expression> result = new ArrayList<>();
		if (pctx == null)
			return result;
		if (pctx.namedexpr_test() != null || pctx.star_expr() != null)
			result.add(visitTestlistFirstElement(pctx));
		if (pctx.testOrStar() == null || pctx.testOrStar().size() == 0)
			return result;
		for (TestOrStarContext e : pctx.testOrStar())
			result.add(visitTestOrStar(e));
		return result;
	}

	public List<Expression> extractElementsFromSet(
			DictorsetmakerContext pctx) {
		if (pctx == null)
			return new ArrayList<>();
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < pctx.test().size(); i++)
			result.add(ctx.expr().visitTest(pctx.test(i)));
		return result;
	}

	public List<Pair<Expression, Expression>> extractPairsFromDict(
			DictorsetmakerContext pctx) {
		if (pctx == null)
			return new ArrayList<>();
		List<Pair<Expression, Expression>> result = new ArrayList<>();
		if (pctx.test().size() != 2 * pctx.COLON().size())
			throw new UnsupportedStatementException(
					"We support only initialization of dictonaries in the form of <key> : <value>");
		for (int i = 0; i < pctx.COLON().size(); i++) {
			Expression left = ctx.expr().visitTest(pctx.test(2 * i));
			Expression right = ctx.expr().visitTest(pctx.test(2 * i + 1));
			result.add(Pair.of(left, right));
		}
		return result;
	}
}
