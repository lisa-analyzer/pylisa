package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.antlr.Python3Parser.AddContext;
import it.unive.pylisa.antlr.Python3Parser.And_exprContext;
import it.unive.pylisa.antlr.Python3Parser.And_testContext;
import it.unive.pylisa.antlr.Python3Parser.ArglistContext;
import it.unive.pylisa.antlr.Python3Parser.ArgumentContext;
import it.unive.pylisa.antlr.Python3Parser.Arith_exprContext;
import it.unive.pylisa.antlr.Python3Parser.AtomContext;
import it.unive.pylisa.antlr.Python3Parser.Atom_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_forContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_ifContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_iterContext;
import it.unive.pylisa.antlr.Python3Parser.ComparisonContext;
import it.unive.pylisa.antlr.Python3Parser.DictorsetmakerContext;
import it.unive.pylisa.antlr.Python3Parser.DivContext;
import it.unive.pylisa.antlr.Python3Parser.Encoding_declContext;
import it.unive.pylisa.antlr.Python3Parser.ExprContext;
import it.unive.pylisa.antlr.Python3Parser.FactorContext;
import it.unive.pylisa.antlr.Python3Parser.FloorDivContext;
import it.unive.pylisa.antlr.Python3Parser.LambdefContext;
import it.unive.pylisa.antlr.Python3Parser.Lambdef_nocondContext;
import it.unive.pylisa.antlr.Python3Parser.Left_shiftContext;
import it.unive.pylisa.antlr.Python3Parser.Mat_mulContext;
import it.unive.pylisa.antlr.Python3Parser.MinusContext;
import it.unive.pylisa.antlr.Python3Parser.ModContext;
import it.unive.pylisa.antlr.Python3Parser.MulContext;
import it.unive.pylisa.antlr.Python3Parser.Not_testContext;
import it.unive.pylisa.antlr.Python3Parser.Or_testContext;
import it.unive.pylisa.antlr.Python3Parser.PowerContext;
import it.unive.pylisa.antlr.Python3Parser.Right_shiftContext;
import it.unive.pylisa.antlr.Python3Parser.SliceopContext;
import it.unive.pylisa.antlr.Python3Parser.Star_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Subscript_Context;
import it.unive.pylisa.antlr.Python3Parser.SubscriptlistContext;
import it.unive.pylisa.antlr.Python3Parser.TermContext;
import it.unive.pylisa.antlr.Python3Parser.TestContext;
import it.unive.pylisa.antlr.Python3Parser.TestOrStarContext;
import it.unive.pylisa.antlr.Python3Parser.Test_nocondContext;
import it.unive.pylisa.antlr.Python3Parser.Testlist_compContext;
import it.unive.pylisa.antlr.Python3Parser.TrailerContext;
import it.unive.pylisa.antlr.Python3Parser.Xor_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_exprContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.PyClassUnit;
import java.util.List;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Pair;

/**
 * Root dispatcher for Python expression parsing. Delegates each grammar rule to
 * one of five category sub-visitors introduced in Chunk 3:
 * {@link LiteralVisitor}, {@link BinaryOpVisitor}, {@link AccessVisitor},
 * {@link FunctionalVisitor}, and {@link YieldVisitor}. Cross-category recursion
 * inside those sub-visitors routes back through {@link ParserContext#expr()},
 * so every rule resolves via this dispatcher.
 */
public final class ExpressionVisitor extends Python3ParserBaseVisitor<Object> {

	private final LiteralVisitor literals;
	private final BinaryOpVisitor binops;
	private final AccessVisitor access;
	private final FunctionalVisitor functional;
	private final YieldVisitor yields;

	public ExpressionVisitor(
			ParserContext ctx,
			ParserSupport support) {
		Objects.requireNonNull(ctx);
		Objects.requireNonNull(support);
		this.literals = new LiteralVisitor(ctx, support);
		this.binops = new BinaryOpVisitor(ctx, support);
		this.access = new AccessVisitor(ctx, support);
		this.functional = new FunctionalVisitor(ctx, support);
		this.yields = new YieldVisitor(ctx, support);
	}

	// === literals ===

	@Override
	public Expression visitAtom(
			AtomContext c) {
		return literals.visitAtom(c);
	}

	@Override
	public Expression visitDictorsetmaker(
			DictorsetmakerContext c) {
		return literals.visitDictorsetmaker(c);
	}

	@Override
	public Expression visitTestlist_comp(
			Testlist_compContext c) {
		return literals.visitTestlist_comp(c);
	}

	@Override
	public Expression visitTestOrStar(
			TestOrStarContext c) {
		return literals.visitTestOrStar(c);
	}

	public List<Expression> extractExpressionsFromTestlist_comp(
			Testlist_compContext c) {
		return literals.extractExpressionsFromTestlist_comp(c);
	}

	public List<Expression> extractElementsFromSet(
			DictorsetmakerContext c) {
		return literals.extractElementsFromSet(c);
	}

	public List<Pair<Expression, Expression>> extractPairsFromDict(
			DictorsetmakerContext c) {
		return literals.extractPairsFromDict(c);
	}

	// === binary / unary ops ===

	@Override
	public Expression visitOr_test(
			Or_testContext c) {
		return binops.visitOr_test(c);
	}

	@Override
	public Expression visitAnd_test(
			And_testContext c) {
		return binops.visitAnd_test(c);
	}

	@Override
	public Expression visitNot_test(
			Not_testContext c) {
		return binops.visitNot_test(c);
	}

	@Override
	public Expression visitComparison(
			ComparisonContext c) {
		return binops.visitComparison(c);
	}

	public Expression visitExpr(
			ExprContext c) {
		return binops.visitExpr(c);
	}

	@Override
	public Expression visitXor_expr(
			Xor_exprContext c) {
		return binops.visitXor_expr(c);
	}

	@Override
	public Expression visitAnd_expr(
			And_exprContext c) {
		return binops.visitAnd_expr(c);
	}

	@Override
	public Expression visitLeft_shift(
			Left_shiftContext c) {
		return binops.visitLeft_shift(c);
	}

	@Override
	public Expression visitRight_shift(
			Right_shiftContext c) {
		return binops.visitRight_shift(c);
	}

	@Override
	public Expression visitArith_expr(
			Arith_exprContext c) {
		return binops.visitArith_expr(c);
	}

	@Override
	public Expression visitAdd(
			AddContext c) {
		return binops.visitAdd(c);
	}

	@Override
	public Expression visitMinus(
			MinusContext c) {
		return binops.visitMinus(c);
	}

	@Override
	public Expression visitTerm(
			TermContext c) {
		return binops.visitTerm(c);
	}

	@Override
	public Expression visitMul(
			MulContext c) {
		return binops.visitMul(c);
	}

	public Expression visitMat_mul(
			Mat_mulContext c) {
		return binops.visitMat_mul(c);
	}

	public Expression visitDiv(
			DivContext c) {
		return binops.visitDiv(c);
	}

	public Expression visitMod(
			ModContext c) {
		return binops.visitMod(c);
	}

	public Expression visitFloorDiv(
			FloorDivContext c) {
		return binops.visitFloorDiv(c);
	}

	@Override
	public Expression visitFactor(
			FactorContext c) {
		return binops.visitFactor(c);
	}

	@Override
	public Expression visitPower(
			PowerContext c) {
		return binops.visitPower(c);
	}

	// === access / trailers ===

	@Override
	public Expression visitAtom_expr(
			Atom_exprContext c) {
		return access.visitAtom_expr(c);
	}

	@Override
	public Expression visitArgument(
			ArgumentContext c) {
		return access.visitArgument(c);
	}

	@Override
	public Expression visitSubscript_(
			Subscript_Context c) {
		return access.visitSubscript_(c);
	}

	public List<Expression> extractExpressionsFromSubscriptlist(
			SubscriptlistContext c) {
		return access.extractExpressionsFromSubscriptlist(c);
	}

	public PyClassUnit findEnclosingPyClassUnit() {
		return access.findEnclosingPyClassUnit();
	}

	// === functional (ternary, lambda, comprehensions) ===

	@Override
	public Expression visitTest(
			TestContext c) {
		return functional.visitTest(c);
	}

	@Override
	public Expression visitTest_nocond(
			Test_nocondContext c) {
		return functional.visitTest_nocond(c);
	}

	@Override
	public Expression visitLambdef(
			LambdefContext c) {
		return functional.visitLambdef(c);
	}

	@Override
	public Expression visitLambdef_nocond(
			Lambdef_nocondContext c) {
		return functional.visitLambdef_nocond(c);
	}

	@Override
	public Object visitComp_iter(
			Comp_iterContext c) {
		return functional.visitComp_iter(c);
	}

	@Override
	public Object visitComp_for(
			Comp_forContext c) {
		return functional.visitComp_for(c);
	}

	@Override
	public Object visitComp_if(
			Comp_ifContext c) {
		return functional.visitComp_if(c);
	}

	// === yield / stubs ===

	@Override
	public Object visitYield_expr(
			Yield_exprContext c) {
		return yields.visitYield_expr(c);
	}

	@Override
	public Object visitYield_arg(
			Yield_argContext c) {
		return yields.visitYield_arg(c);
	}

	@Override
	public Object visitStar_expr(
			Star_exprContext c) {
		return yields.visitStar_expr(c);
	}

	@Override
	public Object visitEncoding_decl(
			Encoding_declContext c) {
		return yields.visitEncoding_decl(c);
	}

	@Override
	public Object visitTrailer(
			TrailerContext c) {
		return yields.visitTrailer(c);
	}

	@Override
	public Object visitSubscriptlist(
			SubscriptlistContext c) {
		return yields.visitSubscriptlist(c);
	}

	@Override
	public Object visitSliceop(
			SliceopContext c) {
		return yields.visitSliceop(c);
	}

	public List<Expression> extractExpressionsFromYieldArg(
			Yield_argContext c) {
		return yields.extractExpressionsFromYieldArg(c);
	}

	@Override
	public Object visitArglist(
			ArglistContext c) {
		throw new it.unive.pylisa.UnsupportedStatementException();
	}
}
