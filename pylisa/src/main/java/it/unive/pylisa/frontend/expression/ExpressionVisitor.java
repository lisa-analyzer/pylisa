package it.unive.pylisa.frontend.expression;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.literal.FalseLiteral;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.lisa.program.cfg.statement.literal.TrueLiteral;
import it.unive.lisa.program.cfg.statement.logic.Not;
import it.unive.lisa.program.cfg.statement.numeric.Division;
import it.unive.lisa.program.cfg.statement.numeric.Subtraction;
import it.unive.pylisa.UnsupportedStatementException;
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
import it.unive.pylisa.antlr.Python3Parser.Comp_opContext;
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
import it.unive.pylisa.antlr.Python3Parser.VarargslistContext;
import it.unive.pylisa.antlr.Python3Parser.VfpdefContext;
import it.unive.pylisa.antlr.Python3Parser.Xor_exprContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_argContext;
import it.unive.pylisa.antlr.Python3Parser.Yield_exprContext;
import it.unive.pylisa.antlr.Python3ParserBaseVisitor;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Empty;
import it.unive.pylisa.cfg.expression.LambdaExpression;
import it.unive.pylisa.cfg.expression.ListCreation;
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
import it.unive.pylisa.cfg.expression.PyTernaryOperator;
import it.unive.pylisa.cfg.expression.RangeValue;
import it.unive.pylisa.cfg.expression.SetCreation;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.TupleCreation;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyEllipsisLiteral;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.statement.FunctionApply;
import it.unive.pylisa.cfg.statement.PyNameRef;
import it.unive.pylisa.frontend.ParserContext;
import it.unive.pylisa.frontend.ParserSupport;
import it.unive.pylisa.program.PyClassUnit;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Handles Python expressions. One of three sibling category visitors introduced
 * in Chunk 2; shares state with {@link ParserContext} and cross-dispatches via
 * {@code ctx.stmt()} / {@code ctx.def()}.
 */
public final class ExpressionVisitor extends Python3ParserBaseVisitor<Object> {

	private static final Logger LOG = LogManager.getLogger(ExpressionVisitor.class);

	private final ParserContext ctx;
	private final ParserSupport support;

	public ExpressionVisitor(
			ParserContext ctx,
			ParserSupport support) {
		this.ctx = Objects.requireNonNull(ctx);
		this.support = Objects.requireNonNull(support);
	}

	@Override
	public Expression visitAtom(
			AtomContext pctx) {
		if (pctx.NAME() != null) {
			return support.makeRef(pctx.NAME().getText(), support.getLocation(pctx));
		} else if (pctx.NUMBER() != null) {
			String text = pctx.NUMBER().getText().toLowerCase().replaceAll("_", "");
			if (text.endsWith("j"))
				throw new UnsupportedStatementException(
						"complex numbers are not supported (at " + support.getLocation(pctx) + ")");

			if (text.contains("e") || text.contains("."))
				return new Float32Literal(ctx.currentCFG(), support.getLocation(pctx), Float.parseFloat(text));

			if (text.startsWith("0x"))
				return new Int32Literal(ctx.currentCFG(), support.getLocation(pctx),
						Integer.parseInt(text.substring(2), 16));
			if (text.startsWith("0o"))
				return new Int32Literal(ctx.currentCFG(), support.getLocation(pctx),
						Integer.parseInt(text.substring(2), 8));
			if (text.startsWith("0b"))
				return new Int32Literal(ctx.currentCFG(), support.getLocation(pctx),
						Integer.parseInt(text.substring(2), 2));
			return new Int32Literal(ctx.currentCFG(), support.getLocation(pctx), Integer.parseInt(text));
		} else if (pctx.FALSE() != null)
			return new FalseLiteral(ctx.currentCFG(), support.getLocation(pctx));
		else if (pctx.TRUE() != null)
			return new TrueLiteral(ctx.currentCFG(), support.getLocation(pctx));
		else if (pctx.NONE() != null)
			return new PyNoneLiteral(ctx.currentCFG(), support.getLocation(pctx));
		else if (pctx.STRING().size() > 0)
			return support.strip(support.getLocation(pctx), pctx.STRING(0).getText());
		else if (pctx.yield_expr() != null)
			throw new UnsupportedStatementException();
		else if (pctx.OPEN_BRACE() == null && pctx.dictorsetmaker() != null)
			return visitDictorsetmaker(pctx.dictorsetmaker());
		else if (pctx.OPEN_BRACK() != null) {
			List<Expression> sts = extractExpressionsFromTestlist_comp(pctx.testlist_comp());
			return new ListCreation(ctx.currentCFG(), support.getLocation(pctx), sts.toArray(Expression[]::new));
		} else if (pctx.OPEN_PAREN() != null) {
			if (pctx.yield_expr() != null)
				throw new UnsupportedStatementException("yield expressions not supported");
			List<Expression> sts = extractExpressionsFromTestlist_comp(pctx.testlist_comp());
			if (sts.size() <= 1)
				return sts.isEmpty() ? new TupleCreation(ctx.currentCFG(), support.getLocation(pctx)) : sts.get(0);
			support.unsound(pctx, "tuple creation treated as first element");
			return sts.get(0);
		} else if (pctx.OPEN_BRACE() != null) {
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
		} else if (pctx.ELLIPSIS() != null)
			return new PyEllipsisLiteral(ctx.currentCFG(), support.getLocation(pctx));
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitAtom_expr(
			Atom_exprContext pctx) {
		/*
		 * atom_expr: (AWAIT)? atom trailer*; atom: ('('
		 * (yield_expr|testlist_comp)? ')' | '[' (testlist_comp)? ']' | '{'
		 * (dictorsetmaker)? '}' | NAME | NUMBER | STRING+ | '...' | 'None' |
		 * 'True' | 'False');
		 */

		if (pctx.AWAIT() != null)
			support.unsound(pctx, "await stripped");
		if (pctx.trailer().size() > 0) {
			Expression[] accessChain = new Expression[pctx.trailer().size() + 1];
			int i = 0;
			Expression access = visitAtom(pctx.atom());
			accessChain[i] = access;
			i += 1;
			Expression previous_access = null;
			for (TrailerContext trailer : pctx.trailer()) {
				if (trailer.NAME() != null) {
					// a.b
					String attr = trailer.NAME().getText();
					access = new AttributeAccess(
							ctx.currentCFG(),
							support.getLocation(trailer),
							access,
							attr);
					accessChain[i] = access;
				} else if (trailer.OPEN_PAREN() != null) {

					List<Expression> args = new ArrayList<>();
					boolean receiverWasPrepended = (i >= 2);
					if (receiverWasPrepended) {
						// Function referenced via an object or a class: prepend
						// the receiver.
						args.add(accessChain[i - 2]);
					}
					// ── Python 3 zero-arg super()
					// super() with no arguments is equivalent to
					// super(ClassName, self). Synthesise the two-arg form so
					// the pluggable Super semantics can resolve the parent
					// type at analysis time.
					if (access instanceof PyNameRef superRef
							&& "super".equals(superRef.getName())
							&& trailer.arglist() == null
							&& !receiverWasPrepended
							&& ctx.objectUnit() != null) {
						PyClassUnit enclosingClass = findEnclosingPyClassUnit();
						if (enclosingClass != null) {
							String fullName = enclosingClass.getName();
							String simpleName = fullName.contains(".")
									? fullName.substring(fullName.lastIndexOf('.') + 1)
									: fullName;
							Expression classRef = support.makeRef(simpleName, support.getLocation(trailer));
							LOG.debug("super() synthesis in '{}': simpleName='{}', classRef='{}' id={}",
									ctx.currentUnit().getName(), simpleName, classRef,
									System.identityHashCode(classRef));
							String firstParamName = ctx.currentCFG().getDescriptor().getFormals().length > 0
									? ctx.currentCFG().getDescriptor().getFormals()[0].getName()
									: "self";
							Expression selfRef = new VariableRef(ctx.currentCFG(), support.getLocation(trailer),
									firstParamName);
							Expression superFunc = support.makeScopedAttributeRef(ctx.objectUnit(), "super",
									support.getLocation(trailer));
							access = new FunctionApply(ctx.currentCFG(), support.getLocation(trailer), superFunc,
									new Expression[] { classRef, selfRef }, false);
							LOG.debug("super() FunctionApply created: id={}", System.identityHashCode(access));
							accessChain[i] = selfRef; // original self as
														// receiver;
														// super proxy only for
														// type
														// resolution

							i++;
							continue;
						}
					}
					if (trailer.arglist() != null)
						for (ArgumentContext arg : trailer.arglist().argument())
							args.add(visitArgument(arg));

					args = support.convertAssignmentsToByNameParameters(args);
					access = new FunctionApply(
							ctx.currentCFG(),
							support.getLocation(trailer),
							access,
							args.toArray(Expression[]::new),
							receiverWasPrepended);
					accessChain[i] = access;
				} else if (trailer.OPEN_BRACK() != null) {
					// expr[key] → expr.__getitem__(key)
					List<Subscript_Context> subs = trailer.subscriptlist().subscript_();
					if (subs.size() == 1) {
						Expression key = visitSubscript_(subs.get(0));
						Expression receiver = access;
						Expression getitemAttr = new AttributeAccess(
								ctx.currentCFG(), support.getLocation(trailer), receiver, "__getitem__");
						access = new FunctionApply(
								ctx.currentCFG(), support.getLocation(trailer), getitemAttr,
								new Expression[] { receiver, key }, true);
					} else {
						support.unsound(trailer, "multiple subscripts not supported");
					}
					accessChain[i] = access;
				}
				i++;
			}
			return access;
		} else
			return visitAtom(pctx.atom());
	}

	@Override
	public Expression visitTest(
			TestContext pctx) {
		if (pctx.IF() != null) {
			Expression trueCase = visitOr_test(pctx.or_test(0));
			Expression booleanGuard = visitOr_test(pctx.or_test(1));
			Expression falseCase = visitTest(pctx.test());

			return new PyTernaryOperator(ctx.currentCFG(), support.getLocation(pctx), booleanGuard,
					trueCase, falseCase);
		} else if (pctx.lambdef() != null)
			return visitLambdef(pctx.lambdef());
		else
			return visitOr_test(pctx.or_test(0));
	}

	@Override
	public Expression visitTest_nocond(
			Test_nocondContext pctx) {
		if (pctx.or_test() != null)
			return visitOr_test(pctx.or_test());
		else
			return visitLambdef_nocond(pctx.lambdef_nocond());
	}

	@Override
	public Expression visitLambdef(
			LambdefContext pctx) {
		List<Expression> args;
		if (pctx.varargslist() != null)
			args = extractNamesFromVarArgList(pctx.varargslist());
		else
			args = new ArrayList<>();

		Expression body = visitTest(pctx.test());
		return new LambdaExpression(args, body, ctx.currentCFG(), support.getLocation(pctx));
	}

	@Override
	public Expression visitLambdef_nocond(
			Lambdef_nocondContext pctx) {
		List<Expression> args;
		if (pctx.varargslist() != null)
			args = extractNamesFromVarArgList(pctx.varargslist());
		else
			args = new ArrayList<>();

		Expression body = visitTest_nocond(pctx.test_nocond());
		return new LambdaExpression(args, body, ctx.currentCFG(), support.getLocation(pctx));
	}

	@Override
	public Expression visitOr_test(
			Or_testContext pctx) {
		return support.foldBinaryOp(pctx.and_test(), this::visitAnd_test, PyOr::new, support.getLocation(pctx));
	}

	@Override
	public Expression visitAnd_test(
			And_testContext pctx) {
		return support.foldBinaryOp(pctx.not_test(), this::visitNot_test, PyAnd::new, support.getLocation(pctx));
	}

	@Override
	public Expression visitNot_test(
			Not_testContext pctx) {
		if (pctx.NOT() != null)
			return new Not(ctx.currentCFG(), support.getLocation(pctx), visitNot_test(pctx.not_test()));
		else
			return visitComparison(pctx.comparison());
	}

	@Override
	public Expression visitComparison(
			ComparisonContext pctx) {
		int nExpr = pctx.expr().size();
		Expression result = null;
		switch (nExpr) {
		case 0:
			throw new UnsupportedStatementException();
		case 1:
			result = visitExpr(pctx.expr(0));
			break;
		case 2:
		default:
			Comp_opContext operator = pctx.comp_op(0);
			Expression left = visitExpr(pctx.expr(0));
			Expression right = visitExpr(pctx.expr(1));
			if (operator.EQUALS() != null)
				result = new PyEquals(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.GREATER_THAN() != null)
				result = new PyGreaterThan(ctx.currentCFG(), support.getLocation(pctx), left, right);
			if (operator.GT_EQ() != null)
				result = new PyGreaterOrEqual(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.NOT() != null && operator.IN() != null)
				result = new Not(ctx.currentCFG(), support.getLocation(pctx),
						new PyIn(ctx.currentCFG(), support.getLocation(pctx), left, right));
			else if (operator.IN() != null)
				result = new PyIn(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.IS() != null && operator.NOT() != null)
				result = new Not(ctx.currentCFG(), support.getLocation(pctx),
						new PyIs(ctx.currentCFG(), support.getLocation(pctx), left, right));
			else if (operator.IS() != null)
				result = new PyIs(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.LESS_THAN() != null)
				result = new PyLessThan(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.LT_EQ() != null)
				result = new PyLessOrEqual(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.NOT_EQ_1() != null)
				result = new PyNotEqual(ctx.currentCFG(), support.getLocation(pctx), left, right);

			if (operator.NOT_EQ_2() != null)
				result = new PyNotEqual(ctx.currentCFG(), support.getLocation(pctx), left, right);
			break;
		}

		return result;
	}

	public Expression visitExpr(
			ExprContext pctx) {
		return support.foldBinaryOp(pctx.xor_expr(), this::visitXor_expr, PyBitwiseOr::new,
				support.getLocation(pctx));
	}

	@Override
	public Expression visitXor_expr(
			Xor_exprContext pctx) {
		return support.foldBinaryOp(pctx.and_expr(), this::visitAnd_expr, PyBitwiseXor::new,
				support.getLocation(pctx));
	}

	@Override
	public Expression visitAnd_expr(
			And_exprContext pctx) {
		return support.foldBinaryOp(pctx.left_shift(), this::visitLeft_shift, PyBitwiseAnd::new,
				support.getLocation(pctx));
	}

	@Override
	public Expression visitLeft_shift(
			Left_shiftContext pctx) {
		int nShift = pctx.left_shift().size() + 1;
		if (nShift == 1)
			return visitRight_shift(pctx.right_shift());
		else if (nShift == 2)
			return new PyBitwiseLeftShift(ctx.currentCFG(), support.getLocation(pctx),
					visitRight_shift(pctx.right_shift()),
					visitLeft_shift(pctx.left_shift(0)));
		else {
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
	}

	@Override
	public Expression visitRight_shift(
			Right_shiftContext pctx) {
		int nShift = pctx.right_shift().size() + 1;
		if (nShift == 1)
			return visitArith_expr(pctx.arith_expr());
		else if (nShift == 2)
			return new PyBitwiseRIghtShift(ctx.currentCFG(), support.getLocation(pctx),
					visitArith_expr(pctx.arith_expr()),
					visitRight_shift(pctx.right_shift(0)));
		else {
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
	}

	@Override
	public Expression visitArith_expr(
			Arith_exprContext pctx) {
		if (pctx.minus() != null)
			return visitMinus(pctx.minus());
		else if (pctx.add() != null)
			return visitAdd(pctx.add());
		else
			return visitTerm(pctx.term());
	}

	@Override
	public Expression visitMinus(
			MinusContext pctx) {
		if (pctx.arith_expr() == null)
			return visitTerm(pctx.term());
		else
			return new Subtraction(ctx.currentCFG(), support.getLocation(pctx),
					visitTerm(pctx.term()),
					visitArith_expr(pctx.arith_expr()));
	}

	@Override
	public Expression visitAdd(
			AddContext pctx) {
		if (pctx.arith_expr() == null)
			return visitTerm(pctx.term());
		else
			return new PyAddition(ctx.currentCFG(), support.getLocation(pctx),
					visitTerm(pctx.term()),
					visitArith_expr(pctx.arith_expr()));
	}

	@Override
	public Expression visitTerm(
			TermContext pctx) {
		if (pctx.mul() != null)
			return visitMul(pctx.mul());
		else if (pctx.mat_mul() != null)
			return visitMat_mul(pctx.mat_mul());
		else if (pctx.div() != null)
			return visitDiv(pctx.div());
		else if (pctx.mod() != null)
			return visitMod(pctx.mod());
		else if (pctx.floorDiv() != null)
			return visitFloorDiv(pctx.floorDiv());
		else if (pctx.factor() != null)
			return visitFactor(pctx.factor());
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitMul(
			MulContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		else
			return new PyMultiplication(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()),
					visitTerm(pctx.term()));
	}

	public Expression visitMat_mul(
			Mat_mulContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		else
			return new PyMatMul(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()),
					visitTerm(pctx.term()));
	}

	public Expression visitDiv(
			DivContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		else
			return new Division(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()),
					visitTerm(pctx.term()));
	}

	public Expression visitMod(
			ModContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		else
			return new PyRemainder(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()),
					visitTerm(pctx.term()));
	}

	public Expression visitFloorDiv(
			FloorDivContext pctx) {
		if (pctx.term() == null)
			return visitFactor(pctx.factor());
		else
			return new PyFloorDiv(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()),
					visitTerm(pctx.term()));
	}

	@Override
	public Expression visitFactor(
			FactorContext pctx) {
		if (pctx.power() != null)
			return visitPower(pctx.power());
		else if (pctx.NOT_OP() != null)
			return new PyBitwiseNot(ctx.currentCFG(), support.getLocation(pctx),
					visitFactor(pctx.factor()));
		else if (pctx.MINUS() != null)
			return new PyMultiplication(ctx.currentCFG(), support.getLocation(pctx),
					new Int32Literal(ctx.currentCFG(), support.getLocation(pctx), -1),
					visitFactor(pctx.factor()));
		return visitFactor(pctx.factor());
	}

	@Override
	public Expression visitPower(
			PowerContext pctx) {
		if (pctx.POWER() != null)
			return new PyPower(ctx.currentCFG(), support.getLocation(pctx),
					visitAtom_expr(pctx.atom_expr()),
					visitFactor(pctx.factor()));
		else
			return visitAtom_expr(pctx.atom_expr());
	}

	@Override
	public Expression visitArgument(
			ArgumentContext pctx) {
		if (pctx.ASSIGN() != null) {
			boolean prev = ctx.shouldPrependUnitAccess();
			ctx.shouldPrependUnitAccess(false);
			Expression left = visitTest(pctx.test(0));
			ctx.shouldPrependUnitAccess(prev);
			Expression right = visitTest(pctx.test(1));
			return new it.unive.pylisa.cfg.expression.PyAssign(ctx.currentCFG(), support.getLocation(pctx), left,
					right);
		} else if (pctx.STAR() != null)
			return new StarExpression(ctx.currentCFG(), support.getLocation(pctx), visitTest(pctx.test(0)));
		else if (pctx.comp_for() != null || pctx.POWER() != null || pctx.test().size() != 1)
			return new Empty(ctx.currentCFG(), support.getLocation(pctx));
		else
			return visitTest(pctx.test(0));
	}

	@Override
	public Object visitArglist(
			ArglistContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitDictorsetmaker(
			DictorsetmakerContext pctx) {
		if (pctx.COLON().size() == 0) {
			List<Expression> values = new ArrayList<>();
			for (TestContext exp : pctx.test())
				values.add(visitTest(exp));
			return new SetCreation(ctx.currentCFG(), support.getLocation(pctx),
					values.toArray(Expression[]::new));
		} else
			throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitSubscript_(
			Subscript_Context pctx) {
		if (pctx.COLON() != null) {
			SourceCodeLocation loc = support.getLocation(pctx);
			Expression left = pctx.test1() == null ? new Empty(ctx.currentCFG(), loc)
					: visitTest(pctx.test1().test());
			Expression middle = pctx.test2() == null ? new Empty(ctx.currentCFG(), loc)
					: visitTest(pctx.test2().test());
			Expression right = pctx.sliceop() == null || pctx.sliceop().test() == null
					? new Empty(ctx.currentCFG(), loc)
					: visitTest(pctx.sliceop().test());
			return new RangeValue(ctx.currentCFG(), loc, left, middle, right);
		} else
			return visitTest(pctx.test());
	}

	@Override
	public Object visitSliceop(
			SliceopContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitTestlist_comp(
			Testlist_compContext pctx) {
		if (pctx.comp_for() != null)
			throw new UnsupportedStatementException();
		return visitTestOrStar(pctx.testOrStar(0));
	}

	@Override
	public Expression visitTestOrStar(
			TestOrStarContext pctx) {
		if (pctx.star_expr() != null)
			throw new UnsupportedStatementException();
		return visitTest(pctx.test());
	}

	@Override
	public Object visitTrailer(
			TrailerContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitSubscriptlist(
			SubscriptlistContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_iter(
			Comp_iterContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_for(
			Comp_forContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_if(
			Comp_ifContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitYield_expr(
			Yield_exprContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitYield_arg(
			Yield_argContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitEncoding_decl(
			Encoding_declContext pctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitStar_expr(
			Star_exprContext pctx) {
		throw new UnsupportedStatementException();
	}

	// === helpers (used by this class and other sibling visitors) ===

	/**
	 * Returns the {@link PyClassUnit} that lexically encloses the current
	 * function being parsed, or {@code null} if the current function is not
	 * defined inside a Python class.
	 */
	public PyClassUnit findEnclosingPyClassUnit() {
		String funcName = ctx.currentUnit().getName();
		int lastDot = funcName.lastIndexOf('.');
		if (lastDot <= 0)
			return null;
		String classUnitName = funcName.substring(0, lastDot);
		it.unive.lisa.program.Unit candidate = ctx.program().getUnit(classUnitName);
		LOG.debug("findEnclosingPyClassUnit: funcName='{}', classUnitName='{}', candidate={}",
				funcName, classUnitName, candidate == null ? "null" : candidate.getName());
		if (candidate instanceof PyClassUnit pcu)
			return pcu;
		return null;
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
			Expression left = visitTest(pctx.test(2 * i));
			Expression right = visitTest(pctx.test(2 * i + 1));
			result.add(Pair.of(left, right));
		}
		return result;
	}

	public List<Expression> extractElementsFromSet(
			DictorsetmakerContext pctx) {
		if (pctx == null)
			return new ArrayList<>();
		List<Expression> result = new ArrayList<>();
		for (int i = 0; i < pctx.test().size(); i++)
			result.add(visitTest(pctx.test(i)));
		return result;
	}

	public List<Expression> extractExpressionsFromYieldArg(
			Yield_argContext pctx) {
		if (pctx.test() != null) {
			List<Expression> r = new ArrayList<>(1);
			r.add(visitTest(pctx.test()));
			return r;
		} else
			return ctx.stmt().visitTestlist(pctx.testlist());
	}

	public List<Expression> extractExpressionsFromSubscriptlist(
			SubscriptlistContext pctx) {
		List<Expression> result = new ArrayList<>();
		if (pctx.subscript_().size() == 0)
			return result;
		for (Subscript_Context e : pctx.subscript_())
			result.add(visitSubscript_(e));
		return result;
	}

	public List<Expression> extractExpressionsFromTestlist_comp(
			Testlist_compContext pctx) {
		List<Expression> result = new ArrayList<>();
		if (pctx == null || pctx.testOrStar() == null || pctx.testOrStar().size() == 0)
			return result;
		for (TestOrStarContext e : pctx.testOrStar())
			result.add(visitTestOrStar(e));
		return result;
	}

	private List<Expression> extractNamesFromVarArgList(
			VarargslistContext varargslist) {
		List<VfpdefContext> names = varargslist.vfpdef();
		List<Expression> result = new ArrayList<>();
		if (names.size() == 0)
			return result;
		for (VfpdefContext e : names)
			result.add(new VariableRef(ctx.currentCFG(), support.getLocation(e),
					ctx.def().visitVfpdef(e)));
		return result;
	}
}
