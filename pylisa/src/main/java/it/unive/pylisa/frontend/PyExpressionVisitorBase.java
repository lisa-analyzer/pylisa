package it.unive.pylisa.frontend;

import it.unive.lisa.program.*;
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
import it.unive.pylisa.cfg.expression.*;
import it.unive.pylisa.cfg.expression.comparison.PyAnd;
import it.unive.pylisa.cfg.expression.comparison.PyEquals;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyGreaterThan;
import it.unive.pylisa.cfg.expression.comparison.PyLessOrEqual;
import it.unive.pylisa.cfg.expression.comparison.PyLessThan;
import it.unive.pylisa.cfg.expression.comparison.PyNotEqual;
import it.unive.pylisa.cfg.expression.comparison.PyOr;
import it.unive.pylisa.cfg.expression.literal.PyNoneLiteral;
import it.unive.pylisa.cfg.statement.*;
import it.unive.pylisa.program.PyClassUnit;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.lang3.tuple.Pair;

public abstract class PyExpressionVisitorBase extends PyFrontendBase {

	public PyExpressionVisitorBase(
			String filePath,
			boolean notebook) {
		super(filePath, notebook);
	}

	public PyExpressionVisitorBase(
			String filePath,
			boolean notebook,
			Integer... cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	public PyExpressionVisitorBase(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		super(filePath, notebook, cellOrder);
	}

	@Override
	public Expression visitAtom(
			AtomContext ctx) {
		if (ctx.NAME() != null) {
			return makeRef(ctx.NAME().getText(), getLocation(ctx));
		} else if (ctx.NUMBER() != null) {
			String text = ctx.NUMBER().getText().toLowerCase().replaceAll("_", "");
			if (text.endsWith("j"))
				// complex number
				throw new UnsupportedStatementException(
						"complex numbers are not supported (at " + getLocation(ctx) + ")");

			if (text.contains("e") || text.contains("."))
				// floating point
				return new Float32Literal(currentCFG, getLocation(ctx), Float.parseFloat(text));

			// integer
			if (text.startsWith("0x"))
				return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text.substring(2), 16));
			if (text.startsWith("0o"))
				return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text.substring(2), 8));
			if (text.startsWith("0b"))
				return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text.substring(2), 2));
			return new Int32Literal(currentCFG, getLocation(ctx), Integer.parseInt(text));
		} else if (ctx.FALSE() != null)
			// create a literal false
			return new FalseLiteral(currentCFG, getLocation(ctx));
		else if (ctx.TRUE() != null)
			// create a literal true
			return new TrueLiteral(currentCFG, getLocation(ctx));
		else if (ctx.NONE() != null)
			// create a literal false
			return new PyNoneLiteral(currentCFG, getLocation(ctx));
		else if (ctx.STRING().size() > 0)
			// create a string
			return strip(getLocation(ctx), ctx.STRING(0).getText());
		else if (ctx.yield_expr() != null)
			// yield not supported
			throw new UnsupportedStatementException();
		else if (ctx.OPEN_BRACE() == null && ctx.dictorsetmaker() != null)
			return visitDictorsetmaker(ctx.dictorsetmaker());
		else if (ctx.OPEN_BRACK() != null) {
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			return new ListCreation(currentCFG, getLocation(ctx), sts.toArray(Expression[]::new));
		} else if (ctx.OPEN_PAREN() != null) {
			if (ctx.yield_expr() != null)
				throw new UnsupportedStatementException("yield expressions not supported");
			List<Expression> sts = extractExpressionsFromTestlist_comp(ctx.testlist_comp());
			TupleCreation tupleCreation = new TupleCreation(currentCFG, getLocation(ctx),
					sts.toArray(Expression[]::new));
			if (tupleCreation.getSubExpressions().length == 1)
				return tupleCreation.getSubExpressions()[0];
			return tupleCreation;
		} else if (ctx.OPEN_BRACE() != null) {
			// check if it is a dict or a set
			if (!isADict(ctx.dictorsetmaker())) {
				List<Expression> values = extractElementsFromSet(ctx.dictorsetmaker());
				SetCreation s = new SetCreation(currentCFG, getLocation(ctx), values.toArray(Expression[]::new));
				return s;
			}

			List<Pair<Expression, Expression>> values = extractPairsFromDict(ctx.dictorsetmaker());
			@SuppressWarnings("unchecked")
			DictionaryCreation r = new DictionaryCreation(currentCFG, getLocation(ctx),
					values.toArray(Pair[]::new));
			return r;
		} else if (ctx.ELLIPSIS() != null)
			throw new UnsupportedStatementException();
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitAtom_expr(
			Atom_exprContext ctx) {
		/*
		 * atom_expr: (AWAIT)? atom trailer*; atom: ('('
		 * (yield_expr|testlist_comp)? ')' | '[' (testlist_comp)? ']' | '{'
		 * (dictorsetmaker)? '}' | NAME | NUMBER | STRING+ | '...' | 'None' |
		 * 'True' | 'False');
		 */

		if (ctx.AWAIT() != null)
			unsound(ctx, "await stripped");
		if (ctx.trailer().size() > 0) {
			Expression[] accessChain = new Expression[ctx.trailer().size() + 1];
			int i = 0;
			// trailer: '(' (arglist)? ')' | '[' subscriptlist ']' | '.' NAME;
			Expression access = visitAtom(ctx.atom());
			accessChain[i] = access;
			i += 1;
			String last_name = access instanceof VariableRef ? ((VariableRef) access).getName() : null;
			Expression previous_access = null;
			for (TrailerContext trailer : ctx.trailer()) {
				if (trailer.NAME() != null) {
					// a.b
					String attr = trailer.NAME().getText();
					access = new AttributeAccess(
							currentCFG,
							getLocation(trailer),
							access,
							attr);
					accessChain[i] = access;
				} else if (trailer.OPEN_PAREN() != null) {

					List<Expression> args = new ArrayList<>();
					boolean receiverWasPrepended = (i >= 2);
					if (receiverWasPrepended) {
						// this is function referenced by an object or a
						// class. We need to prepend the receiver.
						args.add(accessChain[i - 2]);
					}
					// ── Python 3 zero-arg super()
					// ────────────────────────────────────
					// super() with no arguments is equivalent to
					// super(ClassName, self).
					// We synthesise the two-arg form here by injecting the
					// enclosing
					// class name and the self parameter so the pluggable Super
					// semantics
					// can resolve the parent type at analysis time.
					if (access instanceof PyNameRef superRef
							&& "super".equals(superRef.getName())
							&& trailer.arglist() == null
							&& !receiverWasPrepended
							&& objectUnit != null) {
						PyClassUnit enclosingClass = findEnclosingPyClassUnit();
						if (enclosingClass != null) {
							String fullName = enclosingClass.getName();
							String simpleName = fullName.contains(".")
									? fullName.substring(fullName.lastIndexOf('.') + 1)
									: fullName;
							Expression classRef = makeRef(simpleName, getLocation(trailer));
							log.debug("super() synthesis in '{}': simpleName='{}', classRef='{}' id={}",
									currentUnit.getName(), simpleName, classRef, System.identityHashCode(classRef));
							String firstParamName = currentCFG.getDescriptor().getFormals().length > 0
								? currentCFG.getDescriptor().getFormals()[0].getName()
								: "self";
						Expression selfRef = new VariableRef(currentCFG, getLocation(trailer), firstParamName);
							Expression superFunc = makeScopedAttributeRef(objectUnit, "super", getLocation(trailer));
							access = new FunctionApply(currentCFG, getLocation(trailer), superFunc,
									new Expression[] { classRef, selfRef }, false);
							log.debug("super() FunctionApply created: id={}", System.identityHashCode(access));
							accessChain[i] = selfRef;  // ← FIX: original self used as receiver, super proxy only for type resolution

							i++;
							continue;
						}
					}
					if (trailer.arglist() != null)
						for (ArgumentContext arg : trailer.arglist().argument())
							args.add(visitArgument(arg));

					args = convertAssignmentsToByNameParameters(args);
					access = new FunctionApply(
							currentCFG,
							getLocation(trailer),
							access,
							args.toArray(Expression[]::new),
							receiverWasPrepended);
					accessChain[i] = access;
				}
				i++;
			}
			return access;
		} else
			return visitAtom(ctx.atom());
	}

	@Override
	public Expression visitTest(
			TestContext ctx) {
		// no if into the condition
		if (ctx.IF() != null) {
			// visit the if into the condition
			Expression trueCase = visitOr_test(ctx.or_test(0));
			Expression booleanGuard = visitOr_test(ctx.or_test(1));
			Expression falseCase = visitTest(ctx.test());

			PyTernaryOperator ternary = new PyTernaryOperator(currentCFG, getLocation(ctx), booleanGuard, trueCase,
					falseCase);

			return ternary;
		} else if (ctx.lambdef() != null)
			return visitLambdef(ctx.lambdef());
		else
			return visitOr_test(ctx.or_test(0));
	}

	@Override
	public Expression visitTest_nocond(
			Test_nocondContext ctx) {
		if (ctx.or_test() != null)
			return visitOr_test(ctx.or_test());
		else
			return visitLambdef_nocond(ctx.lambdef_nocond());

	}

	@Override
	public Expression visitLambdef(
			LambdefContext ctx) {
		List<Expression> args;
		if (ctx.varargslist() != null)
			args = extractNamesFromVarArgList(ctx.varargslist());
		else
			args = new ArrayList<Expression>();

		Expression body = visitTest(ctx.test());
		return new LambdaExpression(
				args,
				body,
				currentCFG,
				getLocation(ctx));
	}

	@Override
	public Expression visitLambdef_nocond(
			Lambdef_nocondContext ctx) {
		List<Expression> args;
		if (ctx.varargslist() != null)
			args = extractNamesFromVarArgList(ctx.varargslist());
		else
			args = new ArrayList<Expression>();

		Expression body = visitTest_nocond(ctx.test_nocond());
		return new LambdaExpression(
				args,
				body,
				currentCFG,
				getLocation(ctx));
	}

	@Override
	public Expression visitOr_test(
			Or_testContext ctx) {
		return foldBinaryOp(ctx.and_test(), this::visitAnd_test, PyOr::new, getLocation(ctx));
	}

	@Override
	public Expression visitAnd_test(
			And_testContext ctx) {
		return foldBinaryOp(ctx.not_test(), ctx2 -> visitNot_test(ctx2), PyAnd::new, getLocation(ctx));
	}

	@Override
	public Expression visitNot_test(
			Not_testContext ctx) {
		if (ctx.NOT() != null)
			return new Not(currentCFG, getLocation(ctx), visitNot_test(ctx.not_test()));
		else
			return visitComparison(ctx.comparison());
	}

	@Override
	public Expression visitComparison(
			ComparisonContext ctx) {
		int nExpr = ctx.expr().size();
		Expression result = null;
		switch (nExpr) {
		case 0:
			throw new UnsupportedStatementException();
		case 1:
			result = visitExpr(ctx.expr(0));
			break;
		case 2:
		default:
			Comp_opContext operator = ctx.comp_op(0);
			Expression left = visitExpr(ctx.expr(0));
			Expression right = visitExpr(ctx.expr(1));
			if (operator.EQUALS() != null)
				result = new PyEquals(currentCFG, getLocation(ctx), left, right);

			// Python greater (>)
			if (operator.GREATER_THAN() != null) {
				result = new PyGreaterThan(currentCFG, getLocation(ctx), left, right);
			}
			// Python greater equal (>=)
			if (operator.GT_EQ() != null)
				result = new PyGreaterOrEqual(currentCFG, getLocation(ctx), left, right);

			// Python in (in)
			if (operator.IN() != null)
				result = new PyIn(currentCFG, getLocation(ctx), left, right);

			// Python is (is)
			if (operator.IS() != null)
				result = new PyIs(currentCFG, getLocation(ctx), left, right);

			// Python less (<)
			if (operator.LESS_THAN() != null)
				result = new PyLessThan(currentCFG, getLocation(ctx), left, right);

			// Python less equal (<=)
			if (operator.LT_EQ() != null)
				result = new PyLessOrEqual(currentCFG, getLocation(ctx), left, right);

			// Python not (not)
			if (operator.NOT() != null)
				result = new Not(currentCFG, getLocation(ctx), left);

			// Python not equals (<>)
			if (operator.NOT_EQ_1() != null)
				result = new PyNotEqual(currentCFG, getLocation(ctx), left, right);

			// Python not equals (!=)
			if (operator.NOT_EQ_2() != null)
				result = new PyNotEqual(currentCFG, getLocation(ctx), left, right);
			break;
		}

		return result;
	}

	public Expression visitExpr(
			ExprContext ctx) {
		return foldBinaryOp(ctx.xor_expr(), this::visitXor_expr, PyBitwiseOr::new, getLocation(ctx));
	}

	@Override
	public Expression visitXor_expr(
			Xor_exprContext ctx) {
		return foldBinaryOp(ctx.and_expr(), this::visitAnd_expr, PyBitwiseXor::new, getLocation(ctx));
	}

	@Override
	public Expression visitAnd_expr(
			And_exprContext ctx) {
		return foldBinaryOp(ctx.left_shift(), this::visitLeft_shift, PyBitwiseAnd::new, getLocation(ctx));
	}

	@Override
	public Expression visitLeft_shift(
			Left_shiftContext ctx) {
		int nShift = ctx.left_shift().size() + 1;
		if (nShift == 1)
			return visitRight_shift(ctx.right_shift());
		else if (nShift == 2)
			return new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
					visitRight_shift(ctx.right_shift()),
					visitLeft_shift(ctx.left_shift(0)));
		else {
			Expression temp = new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
					visitLeft_shift(ctx.left_shift(nShift - 3)),
					visitLeft_shift(ctx.left_shift(nShift - 2)));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyBitwiseLeftShift(currentCFG, getLocation(ctx),
						visitLeft_shift(ctx.left_shift(--nShift - 1)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitRight_shift(
			Right_shiftContext ctx) {
		int nShift = ctx.right_shift().size() + 1;
		if (nShift == 1)
			return visitArith_expr(ctx.arith_expr());
		else if (nShift == 2)
			return new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
					visitArith_expr(ctx.arith_expr()),
					visitRight_shift(ctx.right_shift(0)));
		else {
			Expression temp = new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
					visitRight_shift(ctx.right_shift(nShift - 3)),
					visitRight_shift(ctx.right_shift(nShift - 2)));
			nShift = nShift - 2;
			// concatenate all the Shift expressions together
			while (nShift > 0) {
				temp = new PyBitwiseRIghtShift(currentCFG, getLocation(ctx),
						visitRight_shift(ctx.right_shift(--nShift - 1)),
						temp);
			}
			return temp;
		}
	}

	@Override
	public Expression visitArith_expr(
			Arith_exprContext ctx) {
		// check if there is minus(-) or an add(+)
		if (ctx.minus() != null)
			return visitMinus(ctx.minus());
		else if (ctx.add() != null)
			return visitAdd(ctx.add());
		else
			return visitTerm(ctx.term());
	}

	@Override
	public Expression visitMinus(
			MinusContext ctx) {
		if (ctx.arith_expr() == null)
			return visitTerm(ctx.term());
		else
			return new Subtraction(currentCFG, getLocation(ctx),
					visitTerm(ctx.term()),
					visitArith_expr(ctx.arith_expr()));
	}

	@Override
	public Expression visitAdd(
			AddContext ctx) {
		if (ctx.arith_expr() == null)
			return visitTerm(ctx.term());
		else
			return new PyAddition(currentCFG, getLocation(ctx),
					visitTerm(ctx.term()),
					visitArith_expr(ctx.arith_expr()));
	}

	@Override
	public Expression visitTerm(
			TermContext ctx) {
		// check what's the operation in the context
		if (ctx.mul() != null)
			return visitMul(ctx.mul());
		else if (ctx.mat_mul() != null)
			return visitMat_mul(ctx.mat_mul());
		else if (ctx.div() != null)
			return visitDiv(ctx.div());
		else if (ctx.mod() != null)
			return visitMod(ctx.mod());
		else if (ctx.floorDiv() != null)
			return visitFloorDiv(ctx.floorDiv());
		else if (ctx.factor() != null)
			return visitFactor(ctx.factor());
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitMul(
			MulContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMultiplication(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitMat_mul(
			Mat_mulContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyMatMul(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitDiv(
			DivContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new Division(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitMod(
			ModContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyRemainder(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	public Expression visitFloorDiv(
			FloorDivContext ctx) {
		if (ctx.term() == null)
			return visitFactor(ctx.factor());
		else
			return new PyFloorDiv(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()),
					visitTerm(ctx.term()));
	}

	@Override
	public Expression visitFactor(
			FactorContext ctx) {
		if (ctx.power() != null)
			return visitPower(ctx.power());
		else if (ctx.NOT_OP() != null)
			return new PyBitwiseNot(currentCFG, getLocation(ctx),
					visitFactor(ctx.factor()));
		else if (ctx.MINUS() != null)
			return new PyMultiplication(currentCFG, getLocation(ctx),
					new Int32Literal(currentCFG, getLocation(ctx), -1),
					visitFactor(ctx.factor()));
		return visitFactor(ctx.factor());
	}

	@Override
	public Expression visitPower(
			PowerContext ctx) {
		if (ctx.POWER() != null)
			return new PyPower(currentCFG, getLocation(ctx),
					visitAtom_expr(ctx.atom_expr()),
					visitFactor(ctx.factor()));
		else
			return visitAtom_expr(ctx.atom_expr());
	}

	@Override
	public Expression visitArgument(
			ArgumentContext ctx) {
		if (ctx.ASSIGN() != null) {
			boolean prevShouldPrependUnitAccess = shouldPrependUnitAccess;
			shouldPrependUnitAccess = false;
			Expression left = visitTest(ctx.test(0));
			shouldPrependUnitAccess = prevShouldPrependUnitAccess;
			Expression right = visitTest(ctx.test(1));
			return new PyAssign(currentCFG, getLocation(ctx), left, right);
		}

		else if (ctx.STAR() != null)
			return new StarExpression(currentCFG, getLocation(ctx), visitTest(ctx.test(0)));
		else if (ctx.comp_for() != null || ctx.POWER() != null || ctx.test().size() != 1)
			return new Empty(currentCFG, getLocation(ctx));
		// throw new UnsupportedStatementException("We support only simple
		// arguments in method calls");
		// return null;
		else
			return visitTest(ctx.test(0));
	}

	@Override
	public Object visitArglist(
			ArglistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitDictorsetmaker(
			DictorsetmakerContext ctx) {
		if (ctx.COLON().size() == 0) {
			List<Expression> values = new ArrayList<>();
			for (TestContext exp : ctx.test())
				values.add(visitTest(exp));
			return new SetCreation(currentCFG, getLocation(ctx), values.toArray(Expression[]::new));
		} else
			throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitSubscript_(
			Subscript_Context ctx) {
		if (ctx.COLON() != null) {
			SourceCodeLocation loc = getLocation(ctx);
			Expression left = ctx.test1() == null ? new Empty(currentCFG, loc)
					: visitTest(ctx.test1().test());
			Expression middle = ctx.test2() == null ? new Empty(currentCFG, loc)
					: visitTest(ctx.test2().test());
			Expression right = ctx.sliceop() == null || ctx.sliceop().test() == null ? new Empty(currentCFG, loc)
					: visitTest(ctx.sliceop().test());
			return new RangeValue(currentCFG, loc, left, middle, right);
		} else
			return visitTest(ctx.test());
	}

	@Override
	public Object visitSliceop(
			SliceopContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Expression visitTestlist_comp(
			Testlist_compContext ctx) {
		if (ctx.comp_for() != null)
			// comp_for is not supported
			throw new UnsupportedStatementException();
		return visitTestOrStar(ctx.testOrStar(0));
	}

	@Override
	public Expression visitTestOrStar(
			TestOrStarContext ctx) {
		if (ctx.star_expr() != null)
			// star expr is not supported
			throw new UnsupportedStatementException();
		return visitTest(ctx.test());
	}

	@Override
	public Object visitTrailer(
			TrailerContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitSubscriptlist(
			SubscriptlistContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_iter(
			Comp_iterContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_for(
			Comp_forContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitComp_if(
			Comp_ifContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitYield_expr(
			Yield_exprContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitYield_arg(
			Yield_argContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitEncoding_decl(
			Encoding_declContext ctx) {
		throw new UnsupportedStatementException();
	}

	@Override
	public Object visitStar_expr(
			Star_exprContext ctx) {
		throw new UnsupportedStatementException();
	}

	/**
	 * Returns the {@link PyClassUnit} that lexically encloses the current
	 * function being parsed, or {@code null} if the current function is not
	 * defined inside a Python class (e.g. it is a module-level function).
	 * <p>
	 * During method-body parsing, {@code currentUnit} is the
	 * {@link it.unive.pylisa.program.FunctionUnit} for the method, not the
	 * enclosing class. The enclosing class name is the prefix obtained by
	 * stripping the last {@code ".<methodName>"} component from the function
	 * unit name. The class is then looked up from the LiSA program.
	 *
	 * @return the enclosing {@link PyClassUnit}, or {@code null}
	 */
	protected PyClassUnit findEnclosingPyClassUnit() {
		String funcName = currentUnit.getName();
		int lastDot = funcName.lastIndexOf('.');
		if (lastDot <= 0)
			return null;
		String classUnitName = funcName.substring(0, lastDot);
		it.unive.lisa.program.Unit candidate = program.getUnit(classUnitName);
		log.debug("findEnclosingPyClassUnit: funcName='{}', classUnitName='{}', candidate={}",
				funcName, classUnitName, candidate == null ? "null" : candidate.getName());
		if (candidate instanceof PyClassUnit pcu)
			return pcu;
		return null;
	}

	private List<Expression> extractNamesFromVarArgList(
			VarargslistContext varargslist) {
		List<VfpdefContext> names = varargslist.vfpdef();
		List<Expression> result = new ArrayList<>();
		if (names.size() == 0)
			return result;
		for (VfpdefContext e : names)
			result.add(new VariableRef(currentCFG, getLocation(e), (String) visitVfpdef(e)));
		return result;
	}
}
