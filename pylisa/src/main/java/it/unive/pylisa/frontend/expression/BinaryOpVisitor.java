package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.logic.Not;
import it.unive.lisa.program.cfg.statement.numeric.Division;
import it.unive.lisa.program.cfg.statement.numeric.Subtraction;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.antlr.Python3Parser.AddContext;
import it.unive.pylisa.antlr.Python3Parser.And_exprContext;
import it.unive.pylisa.antlr.Python3Parser.And_testContext;
import it.unive.pylisa.antlr.Python3Parser.Arith_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Comp_opContext;
import it.unive.pylisa.antlr.Python3Parser.ComparisonContext;
import it.unive.pylisa.antlr.Python3Parser.DivContext;
import it.unive.pylisa.antlr.Python3Parser.ExprContext;
import it.unive.pylisa.antlr.Python3Parser.FactorContext;
import it.unive.pylisa.antlr.Python3Parser.FloorDivContext;
import it.unive.pylisa.antlr.Python3Parser.Left_shiftContext;
import it.unive.pylisa.antlr.Python3Parser.Mat_mulContext;
import it.unive.pylisa.antlr.Python3Parser.MinusContext;
import it.unive.pylisa.antlr.Python3Parser.ModContext;
import it.unive.pylisa.antlr.Python3Parser.MulContext;
import it.unive.pylisa.antlr.Python3Parser.Not_testContext;
import it.unive.pylisa.antlr.Python3Parser.Or_testContext;
import it.unive.pylisa.antlr.Python3Parser.PowerContext;
import it.unive.pylisa.antlr.Python3Parser.Right_shiftContext;
import it.unive.pylisa.antlr.Python3Parser.TermContext;
import it.unive.pylisa.antlr.Python3Parser.Xor_exprContext;
import it.unive.pylisa.cfg.expression.PyAddition;
import it.unive.pylisa.cfg.expression.PyBitwiseAnd;
import it.unive.pylisa.cfg.expression.PyBitwiseLeftShift;
import it.unive.pylisa.cfg.expression.PyBitwiseNot;
import it.unive.pylisa.cfg.expression.PyBitwiseOr;
import it.unive.pylisa.cfg.expression.PyBitwiseRIghtShift;
import it.unive.pylisa.cfg.expression.PyBitwiseXor;
import it.unive.pylisa.cfg.expression.PyFloorDiv;
import it.unive.pylisa.cfg.expression.PyIn;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyMatMul;
import it.unive.pylisa.cfg.expression.PyMultiplication;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PyRemainder;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import java.util.Objects;

/**
 * Handles binary and unary operators: logical, comparison, bitwise, shift,
 * arithmetic, power, factor. Extracted from {@link ExpressionVisitor} in Chunk
 * 3. Cross-visitor recursion (e.g. {@code visitPower → visitAtom_expr}) goes
 * through {@code ctx.expr()}.
 */
public final class BinaryOpVisitor {

	@FunctionalInterface
	private interface BinOpFactory {
		Expression create(
				CFG cfg,
				SourceCodeLocation loc,
				Expression left,
				Expression right);
	}

	private final ParserContext ctx;
	private final ParserSupport support;

	public BinaryOpVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	public Expression visitOr_test(
			Or_testContext pctx) {
		return support.foldBinaryOp(pctx.and_test(), this::visitAnd_test, PyOr::new,
				support.getLocation(pctx));
	}

	public Expression visitAnd_test(
			And_testContext pctx) {
		return support.foldBinaryOp(pctx.not_test(), this::visitNot_test, PyAnd::new,
				support.getLocation(pctx));
	}

	public Expression visitNot_test(
			Not_testContext pctx) {
		if (pctx.NOT() != null)
			return new Not(ctx.currentCFG(), support.getLocation(pctx),
					visitNot_test(pctx.not_test()));
		return visitComparison(pctx.comparison());
	}

	public Expression visitComparison(
			ComparisonContext pctx) {
		int n = pctx.expr().size();
		if (n == 0)
			throw new UnsupportedStatementException();
		if (n == 1)
			return visitExpr(pctx.expr(0));
		return buildComparison(pctx);
	}

	private Expression buildComparison(
			ComparisonContext pctx) {
		Comp_opContext op = pctx.comp_op(0);
		Expression left = visitExpr(pctx.expr(0));
		Expression right = visitExpr(pctx.expr(1));
		SourceCodeLocation loc = support.getLocation(pctx);
		if (op.IN() != null)
			return negateIf(op.NOT() != null, loc,
					new PyIn(ctx.currentCFG(), loc, left, right));
		if (op.IS() != null)
			return negateIf(op.NOT() != null, loc,
					new PyIs(ctx.currentCFG(), loc, left, right));
		return buildSimpleComparison(op, loc, left, right);
	}

	private Expression buildSimpleComparison(
			Comp_opContext op,
			SourceCodeLocation loc,
			Expression left,
			Expression right) {
		BinOpFactory f = null;
		if (op.EQUALS() != null)
			f = PyEquals::new;
		else if (op.NOT_EQ_1() != null || op.NOT_EQ_2() != null)
			f = PyNotEqual::new;
		else if (op.LESS_THAN() != null)
			f = PyLessThan::new;
		else if (op.LT_EQ() != null)
			f = PyLessOrEqual::new;
		else if (op.GREATER_THAN() != null)
			f = PyGreaterThan::new;
		else if (op.GT_EQ() != null)
			f = PyGreaterOrEqual::new;
		if (f == null)
			throw new UnsupportedStatementException();
		return f.create(ctx.currentCFG(), loc, left, right);
	}

	private Expression negateIf(
			boolean negate,
			SourceCodeLocation loc,
			Expression inner) {
		return negate ? new Not(ctx.currentCFG(), loc, inner) : inner;
	}

	public Expression visitExpr(
			ExprContext pctx) {
		return support.foldBinaryOp(pctx.xor_expr(), this::visitXor_expr, PyBitwiseOr::new,
				support.getLocation(pctx));
	}

	public Expression visitXor_expr(
			Xor_exprContext pctx) {
		return support.foldBinaryOp(pctx.and_expr(), this::visitAnd_expr, PyBitwiseXor::new,
				support.getLocation(pctx));
	}

	public Expression visitAnd_expr(
			And_exprContext pctx) {
		return support.foldBinaryOp(pctx.left_shift(), this::visitLeft_shift, PyBitwiseAnd::new,
				support.getLocation(pctx));
	}

	public Expression visitLeft_shift(
			Left_shiftContext pctx) {
		int nShift = pctx.left_shift().size() + 1;
		if (nShift == 1)
			return visitRight_shift(pctx.right_shift());
		if (nShift == 2)
			return new PyBitwiseLeftShift(ctx.currentCFG(), support.getLocation(pctx),
					visitRight_shift(pctx.right_shift()),
					visitLeft_shift(pctx.left_shift(0)));
		Expression temp = new PyBitwiseLeftShift(ctx.currentCFG(), support.getLocation(pctx),
				visitLeft_shift(pctx.left_shift(nShift - 3)),
				visitLeft_shift(pctx.left_shift(nShift - 2)));
		nShift = nShift - 2;
		while (nShift > 0) {
			temp = new PyBitwiseLeftShift(ctx.currentCFG(), support.getLocation(pctx),
					visitLeft_shift(pctx.left_shift(--nShift - 1)),
					temp);
		}
		return temp;
	}

	public Expression visitRight_shift(
			Right_shiftContext pctx) {
		int nShift = pctx.right_shift().size() + 1;
		if (nShift == 1)
			return visitArith_expr(pctx.arith_expr());
		if (nShift == 2)
			return new PyBitwiseRIghtShift(ctx.currentCFG(), support.getLocation(pctx),
					visitArith_expr(pctx.arith_expr()),
					visitRight_shift(pctx.right_shift(0)));
		Expression temp = new PyBitwiseRIghtShift(ctx.currentCFG(), support.getLocation(pctx),
				visitRight_shift(pctx.right_shift(nShift - 3)),
				visitRight_shift(pctx.right_shift(nShift - 2)));
		nShift = nShift - 2;
		while (nShift > 0) {
			temp = new PyBitwiseRIghtShift(ctx.currentCFG(), support.getLocation(pctx),
					visitRight_shift(pctx.right_shift(--nShift - 1)),
					temp);
		}
		return temp;
	}

	public Expression visitArith_expr(
			Arith_exprContext pctx) {
		if (pctx.minus() != null)
			return visitMinus(pctx.minus());
		if (pctx.add() != null)
			return visitAdd(pctx.add());
		return visitTerm(pctx.term());
	}

	public Expression visitMinus(
			MinusContext pctx) {
		if (pctx.arith_expr() == null)
			return visitTerm(pctx.term());
		return new Subtraction(ctx.currentCFG(), support.getLocation(pctx),
				visitTerm(pctx.term()),
				visitArith_expr(pctx.arith_expr()));
	}

	public Expression visitAdd(
			AddContext pctx) {
		if (pctx.arith_expr() == null)
			return visitTerm(pctx.term());
		return new PyAddition(ctx.currentCFG(), support.getLocation(pctx),
				visitTerm(pctx.term()),
				visitArith_expr(pctx.arith_expr()));
	}

	public Expression visitTerm(
			TermContext pctx) {
		if (pctx.mul() != null)
			return visitMul(pctx.mul());
		if (pctx.mat_mul() != null)
			return visitMat_mul(pctx.mat_mul());
		if (pctx.div() != null)
			return visitDiv(pctx.div());
		if (pctx.mod() != null)
			return visitMod(pctx.mod());
		if (pctx.floorDiv() != null)
			return visitFloorDiv(pctx.floorDiv());
		if (pctx.factor() != null)
			return visitFactor(pctx.factor());
		throw new UnsupportedStatementException();
	}

	public Expression visitMul(
			MulContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		return new PyMultiplication(ctx.currentCFG(), support.getLocation(pctx),
				visitFactor(pctx.factor()),
				visitTerm(pctx.term()));
	}

	public Expression visitMat_mul(
			Mat_mulContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		return new PyMatMul(ctx.currentCFG(), support.getLocation(pctx),
				visitFactor(pctx.factor()),
				visitTerm(pctx.term()));
	}

	public Expression visitDiv(
			DivContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		return new Division(ctx.currentCFG(), support.getLocation(pctx),
				visitFactor(pctx.factor()),
				visitTerm(pctx.term()));
	}

	public Expression visitMod(
			ModContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		return new PyRemainder(ctx.currentCFG(), support.getLocation(pctx),
				visitFactor(pctx.factor()),
				visitTerm(pctx.term()));
	}

	public Expression visitFloorDiv(
			FloorDivContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		return new PyFloorDiv(ctx.currentCFG(), support.getLocation(pctx),
				visitFactor(pctx.factor()),
				visitTerm(pctx.term()));
	}

	public Expression visitFactor(
			FactorContext pctx) {
		if (pctx.power() != null)
			return visitPower(pctx.power());
		if (pctx.NOT_OP() != null)
			return new PyBitwiseNot(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()));
		if (pctx.MINUS() != null)
			return new PyMultiplication(ctx.currentCFG(), support.getLocation(pctx),
					new Int32Literal(ctx.currentCFG(), support.getLocation(pctx), -1),
					visitFactor(pctx.factor()));
		return visitFactor(pctx.factor());
	}

	public Expression visitPower(
			PowerContext pctx) {
		if (pctx.POWER() != null)
			return new PyPower(ctx.currentCFG(), support.getLocation(pctx),
					ctx.expr().visitAtom_expr(pctx.atom_expr()),
					visitFactor(pctx.factor()));
		return ctx.expr().visitAtom_expr(pctx.atom_expr());
	}
}
