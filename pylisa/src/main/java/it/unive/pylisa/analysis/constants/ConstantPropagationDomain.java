package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int16Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.Int64Type;
import it.unive.lisa.program.type.Int8Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushInv;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.ArithmeticOperator;
import it.unive.lisa.symbolic.value.operator.DivisionOperator;
import it.unive.lisa.symbolic.value.operator.ModuloOperator;
import it.unive.lisa.symbolic.value.operator.MultiplicationOperator;
import it.unive.lisa.symbolic.value.operator.RemainderOperator;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseAnd;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseOr;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseShiftLeft;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseShiftRight;
import it.unive.lisa.symbolic.value.operator.binary.BitwiseXor;
import it.unive.lisa.symbolic.value.operator.binary.StringContains;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.BitwiseNegation;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.NumericType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.DictPut;
import it.unive.pylisa.symbolic.operators.FloorDivision;
import it.unive.pylisa.symbolic.operators.ListAppend;
import it.unive.pylisa.symbolic.operators.Modulo;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.StringAdd;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import it.unive.pylisa.symbolic.operators.StringMult;
import it.unive.pylisa.symbolic.operators.value.StringFormat;
import it.unive.pylisa.symbolic.operators.value.StringLength;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

/**
 * The domain evaluating expressions over {@link ConstantPropagation} instances.
 * The lattice structure itself lives in {@link ConstantPropagation}.
 */
public class ConstantPropagationDomain
		implements
		BaseNonRelationalValueDomain<ConstantPropagation> {

	@Override
	public ConstantPropagation top() {
		return ConstantPropagation.TOP;
	}

	@Override
	public ConstantPropagation bottom() {
		return ConstantPropagation.BOTTOM;
	}

	private static boolean isAccepted(
			Type t) {
		return t.isNumericType()
				|| t.isStringType()
				|| t.isBooleanType()
				|| t.isNullType();
	}
	
	@Override
	public boolean canProcess(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (expression instanceof PushInv)
			// the type approximation of a pushinv is bottom, so the below check
			// will always fail regardless of the kind of value we are tracking
			return isAccepted(expression.getStaticType());

		Set<Type> rts = null;
		try {
			rts = oracle.getRuntimeTypesOf(expression, pp);
		} catch (SemanticException e) {
			return false;
		}

		if (rts == null || rts.isEmpty())
			// if we have no runtime types, either the type domain has no type
			// information for the given expression (thus it can be anything,
			// also something that we can track) or the computation returned
			// bottom (and the whole state is likely going to go to bottom
			// anyway).
			return true;

		return rts.stream().anyMatch(ConstantPropagationDomain::isAccepted);
	}

	@Override
	public ConstantPropagation evalConstant(
			Constant constant,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (constant.getValue() == null)
			return new ConstantPropagation(new PyNoneConstant(pp.getLocation()));
		if (isAccepted(constant.getStaticType()))
			return new ConstantPropagation(constant);
		return ConstantPropagation.TOP;
	}

	@Override
	public ConstantPropagation evalUnaryExpression(
			UnaryExpression expression,
			ConstantPropagation arg,
			ProgramPoint pp,
			SemanticOracle oracle) {
		UnaryOperator operator = expression.getOperator();
		if (arg.isTop())
			return ConstantPropagation.TOP;
		if (operator == NumericNegation.INSTANCE)
			if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, -1 * arg.as(Integer.class), pp.getLocation()));
			else if (arg.is(Float.class))
				return new ConstantPropagation(
						new Constant(Float32Type.INSTANCE, -1 * arg.as(Float.class), pp.getLocation()));

		if (operator == BitwiseNegation.INSTANCE)
			if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, ~arg.as(Integer.class), pp.getLocation()));

		if (operator == StringLength.INSTANCE)
			if (arg.is(String.class))
				return new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, arg.as(String.class).length(), pp.getLocation()));

		// String constructor
		if (operator == StringConstructor.INSTANCE)
			if (arg.is(String.class))
				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, arg.as(String.class), pp.getLocation()));
			else if (arg.is(Integer.class))
				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, arg.as(Integer.class), pp.getLocation()));
		return ConstantPropagation.TOP;
	}

	@Override
	public ConstantPropagation evalBinaryExpression(
			BinaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle) {
		BinaryOperator operator = expression.getOperator();
		if (operator instanceof ArithmeticOperator) {
			if (left.isTop() || right.isTop() || !left.constant.getStaticType().isNumericType()
					|| !right.constant.getStaticType().isNumericType())
				return ConstantPropagation.TOP;

			Constant c;
			if (operator instanceof AdditionOperator)
				c = sum(left, right, pp);
			else if (operator instanceof DivisionOperator)
				if ((right.is(Integer.class) && right.as(Integer.class) == 0)
						|| (right.is(Float.class) && right.as(Float.class) == 0f))
					return ConstantPropagation.BOTTOM;
				else
					c = div(left, right, pp);
			else if (operator instanceof RemainderOperator || operator instanceof ModuloOperator)
				if ((right.is(Integer.class) && right.as(Integer.class) == 0)
						|| (right.is(Float.class) && right.as(Float.class) == 0f))
					return ConstantPropagation.BOTTOM;
				else
					c = rem(left, right, pp);
			else if (operator instanceof MultiplicationOperator)
				c = mul(left, right, pp);
			else if (operator instanceof SubtractionOperator)
				c = sub(left, right, pp);
			else
				return ConstantPropagation.TOP;
			return new ConstantPropagation(c);
		} else if (operator instanceof Power)
			return power(left, right, pp);
		else if (operator instanceof FloorDivision) {
			if (left.isTop() || right.isTop() || !left.constant.getStaticType().isNumericType()
					|| !right.constant.getStaticType().isNumericType())
				return ConstantPropagation.TOP;
			if ((right.is(Integer.class) && right.as(Integer.class) == 0)
					|| (right.is(Float.class) && right.as(Float.class) == 0f))
				return ConstantPropagation.BOTTOM;
			return new ConstantPropagation(floorDiv(left, right, pp));
		} else if (operator instanceof Modulo) {
			if (left.isTop() || right.isTop())
				return ConstantPropagation.TOP;
			if (left.constant.getStaticType().isStringType())
				return stringFormat(left, right, pp);
			if (!left.constant.getStaticType().isNumericType() || !right.constant.getStaticType().isNumericType())
				return ConstantPropagation.TOP;
			if ((right.is(Integer.class) && right.as(Integer.class) == 0)
					|| (right.is(Float.class) && right.as(Float.class) == 0f))
				return ConstantPropagation.BOTTOM;
			return new ConstantPropagation(pymod(left, right, pp));
		} else if (operator instanceof StringContains)
			return stringContains(left, right, pp);
		else if (operator instanceof StringAdd)
			return stringConcat(left, right, pp);
		else if (operator instanceof StringFormat) {
			return stringFormat(left, right, pp);
		} else if (operator instanceof BitwiseOr)
			return bitwiseOr(left, right, pp);
		else if (operator instanceof BitwiseAnd)
			return bitwiseAnd(left, right, pp);
		else if (operator instanceof BitwiseXor)
			return bitwiseXor(left, right, pp);
		else if (operator instanceof BitwiseShiftLeft)
			return bitwiseLeftShift(left, right, pp);
		else if (operator instanceof BitwiseShiftRight)
			return bitwiseRightShift(left, right, pp);
		if (operator instanceof StringMult)
			return stringRepeat(left, right, pp);
		if (operator instanceof ListAppend)
			return listAppend(left, right, pp);
		return ConstantPropagation.TOP;
	}

	@Override
	public ConstantPropagation evalTernaryExpression(
			TernaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		TernaryOperator operator = expression.getOperator();
		if (operator instanceof DictPut)
			return dictPut(left, middle, right, pp);
		return ConstantPropagation.TOP;
	}

	@Override
	public it.unive.lisa.lattices.Satisfiability satisfiesBinaryExpression(
			BinaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		BinaryOperator operator = expression.getOperator();
		if (!(operator instanceof it.unive.lisa.symbolic.value.operator.ComparisonOperator)
				|| left.isTop() || right.isTop() || left.isBottom() || right.isBottom())
			return it.unive.lisa.lattices.Satisfiability.UNKNOWN;

		Object l = left.getConstant();
		Object r = right.getConstant();

		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonEq.INSTANCE)
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(java.util.Objects.equals(l, r));
		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonNe.INSTANCE)
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(!java.util.Objects.equals(l, r));

		if (!(l instanceof Number) || !(r instanceof Number))
			return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
		double ld = ((Number) l).doubleValue();
		double rd = ((Number) r).doubleValue();

		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonLt.INSTANCE)
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(ld < rd);
		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonLe.INSTANCE)
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(ld <= rd);
		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonGt.INSTANCE)
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(ld > rd);
		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonGe.INSTANCE)
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(ld >= rd);

		return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
	}

	@SuppressWarnings("unchecked")
	private ConstantPropagation dictPut(
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || middle.isTop() || right.isTop()) {
			return ConstantPropagation.TOP;
		}
		DictConstant newdict = new DictConstant(pp.getLocation(), left.as(Map.class), Pair.of(middle, right));
		return new ConstantPropagation(newdict);
	}

	@SuppressWarnings("unchecked")
	private ConstantPropagation listAppend(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop() || !left.is(List.class)) {
			return ConstantPropagation.TOP;
		}

		ListConstant listconst = new ListConstant(pp.getLocation(), left.as(List.class), right);
		return new ConstantPropagation(listconst);
	}

	private ConstantPropagation stringFormat(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {

		if (left.isTop() || right.isTop()) {
			return ConstantPropagation.TOP;
		}
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isStringType()) {
			return new ConstantPropagation(
					new Constant(StringType.INSTANCE, left.as(String.class) + right.as(String.class),
							pp.getLocation()));
		}
		return ConstantPropagation.TOP;
	}

	private Constant div(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class)) {
			Integer l = left.as(Integer.class);
			Integer r = right.as(Integer.class);
			c = l % r == 0
					? new Constant(Int32Type.INSTANCE, l / r, pp.getLocation())
					: new Constant(Float32Type.INSTANCE, l / (float) r, pp.getLocation());
		} else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) / right.as(Integer.class), pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) / right.as(Float.class), pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) / right.as(Float.class), pp.getLocation());
		return c;
	}

	private Constant floorDiv(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		// Python's // floors towards negative infinity, unlike Java's integer
		// division (which truncates towards zero)
		if (left.is(Integer.class) && right.is(Integer.class))
			return new Constant(Int32Type.INSTANCE,
					Math.floorDiv(left.as(Integer.class), right.as(Integer.class)), pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			return new Constant(Float32Type.INSTANCE,
					(float) Math.floor(left.as(Float.class) / right.as(Integer.class)), pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			return new Constant(Float32Type.INSTANCE,
					(float) Math.floor(left.as(Integer.class) / right.as(Float.class)), pp.getLocation());
		else
			return new Constant(Float32Type.INSTANCE,
					(float) Math.floor(left.as(Float.class) / right.as(Float.class)), pp.getLocation());
	}

	private Constant pymod(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		// Python's % takes the sign of the divisor, unlike Java's % (which
		// takes the sign of the dividend): a % b == a - floor(a / b) * b
		if (left.is(Integer.class) && right.is(Integer.class))
			return new Constant(Int32Type.INSTANCE,
					Math.floorMod(left.as(Integer.class), right.as(Integer.class)), pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			return floatPymod(left.as(Float.class), right.as(Integer.class), pp);
		else if (left.is(Integer.class) && right.is(Float.class))
			return floatPymod(left.as(Integer.class), right.as(Float.class), pp);
		else
			return floatPymod(left.as(Float.class), right.as(Float.class), pp);
	}

	private Constant floatPymod(
			float left,
			float right,
			ProgramPoint pp) {
		float result = left - (float) Math.floor(left / right) * right;
		return new Constant(Float32Type.INSTANCE, result, pp.getLocation());
	}

	private Constant rem(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class)) {
			Integer l = left.as(Integer.class);
			Integer r = right.as(Integer.class);
			c = l % r == 0
					? new Constant(Int32Type.INSTANCE, l % r, pp.getLocation())
					: new Constant(Float32Type.INSTANCE, l % (float) r, pp.getLocation());
		} else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) % right.as(Integer.class), pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) % right.as(Float.class), pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) % right.as(Float.class), pp.getLocation());
		return c;
	}

	private Constant sum(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class))
			c = new Constant(Int32Type.INSTANCE, left.as(Integer.class) + right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) + right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) + right.as(Float.class),
					pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) + right.as(Float.class),
					pp.getLocation());
		return c;
	}

	private Constant sub(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class))
			c = new Constant(Int32Type.INSTANCE, left.as(Integer.class) - right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) - right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) - right.as(Float.class),
					pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) - right.as(Float.class),
					pp.getLocation());
		return c;
	}

	private Constant mul(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		Constant c;
		if (left.is(Integer.class) && right.is(Integer.class))
			c = new Constant(Int32Type.INSTANCE, left.as(Integer.class) * right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Float.class) && right.is(Integer.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) * right.as(Integer.class),
					pp.getLocation());
		else if (left.is(Integer.class) && right.is(Float.class))
			c = new Constant(Float32Type.INSTANCE, left.as(Integer.class) * right.as(Float.class),
					pp.getLocation());
		else
			c = new Constant(Float32Type.INSTANCE, left.as(Float.class) * right.as(Float.class),
					pp.getLocation());
		return c;
	}

	private ConstantPropagation stringContains(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isStringType())
			return new ConstantPropagation(
					new Constant(BoolType.INSTANCE, left.as(String.class).contains(right.as(String.class)),
							pp.getLocation()));
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation stringConcat(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {

		if (left.isTop() || right.isTop()) {
			return ConstantPropagation.TOP;
		}
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isStringType()) {
			return new ConstantPropagation(
					new Constant(StringType.INSTANCE, left.as(String.class) + right.as(String.class),
							pp.getLocation()));
		}
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation stringRepeat(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop()) {
			return ConstantPropagation.TOP;
		}
		if (left.constant.getStaticType().isStringType() && right.constant.getStaticType().isNumericType()) {
			if (right.constant.getStaticType().asNumericType().isIntegral()) {
				// use long
				Long longRight = right.as(Integer.class).longValue();
				String stringLeft = left.as(String.class);

				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, stringRepeatAux(stringLeft, longRight), pp.getLocation()));
			}
		}
		if (left.constant.getStaticType().isNumericType() && right.constant.getStaticType().isStringType()) {
			if (left.constant.getStaticType().asNumericType().isIntegral()) {
				// use long
				Long longLeft = left.as(Integer.class).longValue();
				String stringRight = right.as(String.class);

				return new ConstantPropagation(
						new Constant(StringType.INSTANCE, stringRepeatAux(stringRight, longLeft), pp.getLocation()));
			}
		}
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation power(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop()) {
			return ConstantPropagation.TOP;
		}
		// TODO: handle overflow (?)
		if (left.constant.getStaticType().isNumericType() && right.constant.getStaticType().isNumericType()) {
			NumericType superType = left.constant.getStaticType().asNumericType()
					.supertype(right.constant.getStaticType().asNumericType());
			// Class<? extends Number> type = getJavaClassFor(superType);
			if (superType.is8Bits()) {
				return new ConstantPropagation(
						new Constant(Int8Type.INSTANCE,
								(byte) (Math.pow((double) left.as(Byte.class), (double) right.as(Byte.class))),
								pp.getLocation()));
			}
			if (superType.is16Bits()) {
				return new ConstantPropagation(
						new Constant(Int16Type.INSTANCE,
								(short) (Math.pow((double) left.as(Short.class), (double) right.as(Short.class))),
								pp.getLocation()));
			}
			if (superType.is32Bits()) {
				if (!superType.isIntegral()) {
					return new ConstantPropagation(
							new Constant(Float32Type.INSTANCE,
									(float) (Math.pow((double) left.as(Float.class), (double) right.as(Float.class))),
									pp.getLocation()));
				} else {
					if (right.as(Integer.class) < 0) {
						return new ConstantPropagation(
								new Constant(Float32Type.INSTANCE, (float) (Math.pow((double) left.as(Integer.class),
										(double) right.as(Integer.class))), pp.getLocation()));

					} else {
						return new ConstantPropagation(
								new Constant(Int32Type.INSTANCE, (int) (Math.pow((double) left.as(Integer.class),
										(double) right.as(Integer.class))), pp.getLocation()));

					}
				}
			}
			if (superType.is64Bits()) {
				if (!superType.isIntegral()) {
					return new ConstantPropagation(
							new Constant(Float64Type.INSTANCE, Math.pow(left.as(Double.class), right.as(Double.class)),
									pp.getLocation()));
				} else {
					return new ConstantPropagation(
							new Constant(Int64Type.INSTANCE,
									(long) (Math.pow((double) left.as(Long.class), (double) right.as(Long.class))),
									pp.getLocation()));
				}
			}
		}
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation bitwiseOr(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (left.is(Integer.class) && right.is(Integer.class))
			return new ConstantPropagation(
					new Constant(Int32Type.INSTANCE, left.as(Integer.class) | right.as(Integer.class),
							pp.getLocation()));
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation bitwiseAnd(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (left.is(Integer.class) && right.is(Integer.class))
			return new ConstantPropagation(
					new Constant(Int32Type.INSTANCE, left.as(Integer.class) & right.as(Integer.class),
							pp.getLocation()));
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation bitwiseXor(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (left.is(Integer.class) && right.is(Integer.class))
			return new ConstantPropagation(
					new Constant(Int32Type.INSTANCE, left.as(Integer.class) ^ right.as(Integer.class),
							pp.getLocation()));
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation bitwiseLeftShift(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (left.is(Integer.class) && right.is(Integer.class)) {
			int shift = right.as(Integer.class);
			if (shift < 0)
				// Python raises ValueError for a negative shift count
				return ConstantPropagation.BOTTOM;
			return new ConstantPropagation(
					new Constant(Int32Type.INSTANCE, left.as(Integer.class) << shift, pp.getLocation()));
		}
		return ConstantPropagation.TOP;
	}

	private ConstantPropagation bitwiseRightShift(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (left.is(Integer.class) && right.is(Integer.class)) {
			int shift = right.as(Integer.class);
			if (shift < 0)
				// Python raises ValueError for a negative shift count
				return ConstantPropagation.BOTTOM;
			return new ConstantPropagation(
					new Constant(Int32Type.INSTANCE, left.as(Integer.class) >> shift, pp.getLocation()));
		}
		return ConstantPropagation.TOP;
	}

	private String stringRepeatAux(
			String s,
			Long times) {
		StringBuilder sb = new StringBuilder();
		for (long i = 0; i < times; i++) {
			sb.append(s);
		}
		return sb.toString();
	}
}
