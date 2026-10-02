package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
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
import it.unive.pylisa.cfg.expression.PyDivision;
import it.unive.pylisa.cfg.expression.PyFloorDiv;
import it.unive.pylisa.cfg.expression.PyIn;
import it.unive.pylisa.cfg.expression.PyIs;
import it.unive.pylisa.cfg.expression.PyMatMul;
import it.unive.pylisa.cfg.expression.PyMultiplication;
import it.unive.pylisa.cfg.expression.PyNot;
import it.unive.pylisa.cfg.expression.PyPower;
import it.unive.pylisa.cfg.expression.PyRemainder;
import it.unive.pylisa.cfg.expression.PySubtraction;
import it.unive.pylisa.cfg.expression.PyUnaryArithmetic;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyComparisonChain;
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyUnknownLiteral;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGt;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;
import java.util.Arrays;
import java.util.Objects;
import org.antlr.v4.runtime.ParserRuleContext;

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
		Expression folded = support.foldBinaryOp(pctx.and_test(), this::visitAnd_test, PyOr::new,
				support.getLocation(pctx));
		if (pctx.and_test().size() > 1)
			markShortCircuit(pctx, folded);
		return folded;
	}

	public Expression visitAnd_test(
			And_testContext pctx) {
		Expression folded = support.foldBinaryOp(pctx.not_test(), this::visitNot_test, PyAnd::new,
				support.getLocation(pctx));
		if (pctx.not_test().size() > 1)
			markShortCircuit(pctx, folded);
		return folded;
	}

	/**
	 * Marks an {@code and} or {@code or} whose right operand contains a call:
	 * the right operand is evaluated once for each value of the left one, and
	 * only the state of the last evaluation is stored. The fold nests to the
	 * right, so the right operand of the outermost operator holds every later
	 * operand.
	 */
	private void markShortCircuit(
			ParserRuleContext pctx,
			Expression folded) {
		if (ParserSupport.containsCall(((NaryExpression) folded).getSubExpressions()[1]))
			support.limitation(pctx, "call in the right operand of and, or");
	}

	public Expression visitNot_test(
			Not_testContext pctx) {
		if (pctx.NOT() != null)
			return new PyNot(ctx.currentCFG(), support.getLocation(pctx),
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
		if (pctx.comp_op().size() == 1)
			return buildSingleComparison(pctx, pctx.comp_op(0), support.getLocation(pctx));
		BinaryOperator[] operators = new BinaryOperator[pctx.comp_op().size()];
		for (int i = 0; i < operators.length; i++)
			operators[i] = orderingOrEquality(pctx.comp_op(i));
		if (Arrays.stream(operators).allMatch(Objects::nonNull)) {
			Expression[] operands = new Expression[pctx.expr().size()];
			for (int i = 0; i < operands.length; i++)
				operands[i] = visitExpr(pctx.expr(i));
			// each operand after the second is evaluated once for each value of
			// the comparison before it, and only the last state is stored
			for (int i = 2; i < operands.length; i++)
				if (ParserSupport.containsCall(operands[i])) {
					support.limitation(pctx, "call in a chained comparison");
					break;
				}
			return new PyComparisonChain(ctx.currentCFG(), support.getLocation(pctx.comp_op(0)), operands,
					operators);
		}
		// a chain a < b < c holds when every comparison does, and each
		// operand is evaluated once; the comparisons after the first are not
		// built (their operands would be evaluated twice), so the chain is the
		// first comparison and an unknown truth value
		support.limitation(pctx, "chained comparison with in or is (operands after the second are not evaluated)");
		Expression first = buildSingleComparison(pctx, pctx.comp_op(0), support.getLocation(pctx));
		SourceCodeLocation rest = support.getLocation(pctx.comp_op(1));
		return new PyAnd(ctx.currentCFG(), support.getLocation(pctx.comp_op(0)), first,
				new PyUnknownLiteral(ctx.currentCFG(), rest, pctx.getText(), BoolType.INSTANCE));
	}

	/**
	 * Yields the operator of an ordering or equality comparison, or
	 * {@code null} for membership and identity tests.
	 */
	private static BinaryOperator orderingOrEquality(
			Comp_opContext op) {
		if (op.EQUALS() != null)
			return ComparisonEq.INSTANCE;
		if (op.NOT_EQ_1() != null || op.NOT_EQ_2() != null)
			return ComparisonNe.INSTANCE;
		if (op.LESS_THAN() != null)
			return PyComparisonLt.INSTANCE;
		if (op.LT_EQ() != null)
			return PyComparisonLe.INSTANCE;
		if (op.GREATER_THAN() != null)
			return PyComparisonGt.INSTANCE;
		if (op.GT_EQ() != null)
			return PyComparisonGe.INSTANCE;
		return null;
	}

	private Expression buildSingleComparison(
			ComparisonContext pctx,
			Comp_opContext op,
			SourceCodeLocation loc) {
		Expression left = visitExpr(pctx.expr(0));
		Expression right = visitExpr(pctx.expr(1));
		if (op.IN() != null) {
			if (ParserSupport.containsCall(left) || ParserSupport.containsCall(right))
				support.limitation(pctx, "membership test over a call");
			return negateIf(op.NOT() != null, loc,
					new PyIn(ctx.currentCFG(), loc, left, right));
		}
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
		return negate ? new PyNot(ctx.currentCFG(), loc, inner) : inner;
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
		return foldArithmetic(pctx);
	}

	public Expression visitAdd(
			AddContext pctx) {
		return foldArithmetic(pctx);
	}

	/**
	 * Builds a chain of additions and subtractions. The grammar nests the
	 * chain to the right ({@code a - (b - c)}), while Python groups it to the
	 * left ({@code (a - b) - c}): the operands are collected in order and
	 * combined from the left.
	 *
	 * @param link the first link of the chain, a {@link MinusContext} or an
	 *                 {@link AddContext}
	 *
	 * @return the expression
	 */
	private Expression foldArithmetic(
			ParserRuleContext link) {
		Expression result = visitTerm(link.getRuleContext(TermContext.class, 0));
		while (link != null) {
			Arith_exprContext rest = link.getRuleContext(Arith_exprContext.class, 0);
			if (rest == null)
				return result;
			ParserRuleContext next = rest.minus() != null ? rest.minus() : rest.add();
			Expression operand = visitTerm(next != null ? next.getRuleContext(TermContext.class, 0) : rest.term());
			SourceCodeLocation location = support.getLocation(link);
			result = link instanceof MinusContext
					? new PySubtraction(ctx.currentCFG(), location, result, operand)
					: new PyAddition(ctx.currentCFG(), location, result, operand);
			link = next;
		}
		return result;
	}

	public Expression visitTerm(
			TermContext pctx) {
		ParserRuleContext link = termLink(pctx);
		if (link != null)
			return foldTerm(link);
		if (pctx.factor() != null)
			return visitFactor(pctx.factor());
		throw new UnsupportedStatementException();
	}

	public Expression visitMul(
			MulContext pctx) {
		return foldTerm(pctx);
	}

	public Expression visitMat_mul(
			Mat_mulContext pctx) {
		return foldTerm(pctx);
	}

	public Expression visitDiv(
			DivContext pctx) {
		return foldTerm(pctx);
	}

	public Expression visitMod(
			ModContext pctx) {
		return foldTerm(pctx);
	}

	public Expression visitFloorDiv(
			FloorDivContext pctx) {
		return foldTerm(pctx);
	}

	private static ParserRuleContext termLink(
			TermContext term) {
		if (term.mul() != null)
			return term.mul();
		if (term.mat_mul() != null)
			return term.mat_mul();
		if (term.div() != null)
			return term.div();
		if (term.mod() != null)
			return term.mod();
		return term.floorDiv();
	}

	/**
	 * Builds a chain of multiplicative operators, grouped to the left as
	 * Python does (see {@link #foldArithmetic}).
	 *
	 * @param link the first link of the chain
	 *
	 * @return the expression
	 */
	private Expression foldTerm(
			ParserRuleContext link) {
		Expression result = visitFactor(link.getRuleContext(FactorContext.class, 0));
		while (link != null) {
			TermContext rest = link.getRuleContext(TermContext.class, 0);
			if (rest == null)
				return result;
			ParserRuleContext next = termLink(rest);
			Expression operand = visitFactor(
					next != null ? next.getRuleContext(FactorContext.class, 0) : rest.factor());
			result = combineTerm(link, support.getLocation(link), result, operand);
			link = next;
		}
		return result;
	}

	private Expression combineTerm(
			ParserRuleContext link,
			SourceCodeLocation location,
			Expression left,
			Expression right) {
		CFG cfg = ctx.currentCFG();
		if (link instanceof MulContext)
			return new PyMultiplication(cfg, location, left, right);
		if (link instanceof Mat_mulContext) {
			// its operands are objects whose __matmul__ is not run: the result
			// is an unknown value
			support.limitation(link, "matrix multiplication (@)");
			return new PyMatMul(cfg, location, left, right);
		}
		if (link instanceof DivContext)
			return new PyDivision(cfg, location, left, right);
		if (link instanceof ModContext)
			return new PyRemainder(cfg, location, left, right);
		return new PyFloorDiv(cfg, location, left, right);
	}

	public Expression visitFactor(
			FactorContext pctx) {
		if (pctx.power() != null)
			return visitPower(pctx.power());
		if (pctx.NOT_OP() != null)
			return new PyBitwiseNot(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()));
		if (pctx.MINUS() != null)
			return new PyUnaryArithmetic(ctx.currentCFG(), support.getLocation(pctx), visitFactor(pctx.factor()), true);
		return new PyUnaryArithmetic(ctx.currentCFG(), support.getLocation(pctx), visitFactor(pctx.factor()), false);
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
