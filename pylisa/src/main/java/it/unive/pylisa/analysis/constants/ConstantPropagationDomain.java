package it.unive.pylisa.analysis.constants;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
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
import it.unive.lisa.symbolic.value.operator.binary.StringEndsWith;
import it.unive.lisa.symbolic.value.operator.binary.StringIndexOf;
import it.unive.lisa.symbolic.value.operator.binary.StringLastIndexOf;
import it.unive.lisa.symbolic.value.operator.binary.StringStartsWith;
import it.unive.lisa.symbolic.value.operator.ternary.StringReplace;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.BitwiseNegation;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.StringToLowerCase;
import it.unive.lisa.symbolic.value.operator.unary.StringToUpperCase;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.PyBytes;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;
import it.unive.pylisa.symbolic.operators.DictPut;
import it.unive.pylisa.symbolic.operators.FloatPower;
import it.unive.pylisa.symbolic.operators.FloorDivision;
import it.unive.pylisa.symbolic.operators.ListAppend;
import it.unive.pylisa.symbolic.operators.Modulo;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.SliceCreation;
import it.unive.pylisa.symbolic.operators.StringAdd;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import it.unive.pylisa.symbolic.operators.StringMult;
import it.unive.pylisa.symbolic.operators.bytes.BytesLength;
import it.unive.pylisa.symbolic.operators.bytes.BytesOperation;
import it.unive.pylisa.symbolic.operators.bytes.BytesUnary;
import it.unive.pylisa.symbolic.operators.bytes.Codec;
import it.unive.pylisa.symbolic.operators.bytes.CodecRaises;
import it.unive.pylisa.symbolic.operators.bytes.FromHexRaises;
import it.unive.pylisa.symbolic.operators.conversions.ConversionRaises;
import it.unive.pylisa.symbolic.operators.conversions.ToFloat;
import it.unive.pylisa.symbolic.operators.conversions.ToInt;
import it.unive.pylisa.symbolic.operators.conversions.ToRepr;
import it.unive.pylisa.symbolic.operators.strings.ArgPair;
import it.unive.pylisa.symbolic.operators.strings.StrGetItem;
import it.unive.pylisa.symbolic.operators.strings.StrGetSlice;
import it.unive.pylisa.symbolic.operators.strings.StrReplaceCount;
import it.unive.pylisa.symbolic.operators.strings.StrSearch;
import it.unive.pylisa.symbolic.operators.strings.StrStrip;
import it.unive.pylisa.symbolic.operators.value.StringFormat;
import it.unive.pylisa.symbolic.operators.value.StringFormatRaises;
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
				|| t instanceof PyBytesType
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
				// python counts code points, not UTF-16 units
				return new ConstantPropagation(
						new Constant(Int32Type.INSTANCE, PyStrings.length(arg.as(String.class)), pp.getLocation()));

		if (operator == BytesUnary.HEX && arg.is(PyBytes.class))
			return string(PyCodecs.hex(arg.as(PyBytes.class)), pp);
		if ((operator == BytesUnary.UPPER || operator == BytesUnary.LOWER) && arg.is(PyBytes.class))
			return bytes(PyCodecs.asciiCase(arg.as(PyBytes.class), operator == BytesUnary.UPPER), pp);
		if (operator == BytesUnary.FROMHEX && arg.is(String.class)) {
			PyBytes res = PyCodecs.fromHex(arg.as(String.class));
			// ValueError, raised by the caller
			return res == null ? ConstantPropagation.BOTTOM : bytes(res, pp);
		}
		if (operator == BytesUnary.ZEROS) {
			Long n = index(arg);
			if (n == null || n > 1_000_000)
				return ConstantPropagation.TOP;
			// ValueError for negative sizes, raised by the caller
			return n < 0 ? ConstantPropagation.BOTTOM : bytes(new PyBytes(new byte[n.intValue()]), pp);
		}

		if (operator == BytesLength.INSTANCE && arg.is(PyBytes.class))
			return new ConstantPropagation(
					new Constant(Int32Type.INSTANCE, arg.as(PyBytes.class).length(), pp.getLocation()));

		if (operator instanceof StringToUpperCase && arg.is(String.class))
			return string(PyStrings.upper(arg.as(String.class)), pp);
		if (operator instanceof StringToLowerCase && arg.is(String.class))
			return string(PyStrings.lower(arg.as(String.class)), pp);

		// str(x) and repr(x)
		if (operator == StringConstructor.INSTANCE || operator == ToRepr.INSTANCE) {
			String res = arg.constant.getStaticType().isNullType() ? "None"
					: operator == StringConstructor.INSTANCE ? PyPercentFormat.str(arg.getConstant())
							: PyPercentFormat.repr(arg.getConstant(), false);
			return res == null ? ConstantPropagation.TOP : string(res, pp);
		}

		// float(x), whose ValueError is raised by the caller
		if (operator == ToFloat.INSTANCE) {
			Object v = arg.getConstant();
			Double d = v instanceof String ? PyNumbers.parseFloat((String) v)
					: v instanceof Boolean ? ((Boolean) v ? 1.0 : 0.0)
							: v instanceof Number ? ((Number) v).doubleValue() : null;
			if (d == null)
				return v instanceof String ? ConstantPropagation.BOTTOM : ConstantPropagation.TOP;
			return new ConstantPropagation(new Constant(Float32Type.INSTANCE, d.floatValue(), pp.getLocation()));
		}
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
			return power(left, right, operator instanceof FloatPower, pp);
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
		if (operator instanceof BytesOperation)
			return bytesBinary((BytesOperation) operator, left, right, pp);
		if (operator == ToInt.INSTANCE)
			return toInt(left, right, pp);
		if (operator == ArgPair.INSTANCE) {
			if (left.isTop() || right.isTop())
				return ConstantPropagation.TOP;
			return new ConstantPropagation(new Constant(Untyped.INSTANCE,
					new PyStrings.Pair(left.getConstant(), right.getConstant()), pp.getLocation()));
		}
		if (operator == StrGetItem.INSTANCE || operator == StrGetSlice.INSTANCE || operator instanceof StrStrip
				|| operator instanceof StringIndexOf || operator instanceof StringLastIndexOf
				|| operator instanceof StringStartsWith || operator instanceof StringEndsWith)
			return stringBinary(operator, left, right, pp);
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
		if (left.isTop() || middle.isTop() || right.isTop())
			return ConstantPropagation.TOP;

		if (operator == SliceCreation.INSTANCE) {
			Long[] bounds = new Long[3];
			ConstantPropagation[] parts = { left, middle, right };
			for (int i = 0; i < 3; i++) {
				Object v = parts[i].getConstant();
				if (v instanceof RangeBound || parts[i].constant.getStaticType().isNullType())
					// an omitted bound
					bounds[i] = null;
				else if (v instanceof Integer || v instanceof Long)
					bounds[i] = ((Number) v).longValue();
				else if (v instanceof Boolean)
					bounds[i] = (Boolean) v ? 1L : 0L;
				else
					// a TypeError, raised by whoever uses the slice
					return ConstantPropagation.TOP;
			}
			return new ConstantPropagation(new Constant(PyClassType.lookup(LibrarySpecificationProvider.SLICE),
					new PyStrings.Slice(bounds[0], bounds[1], bounds[2]), pp.getLocation()));
		}

		if (operator instanceof StrSearch) {
			boolean isBytes = left.is(PyBytes.class);
			String haystack = text(left), needle = needle(middle, isBytes);
			if (haystack == null || needle == null || !right.is(PyStrings.Slice.class))
				return ConstantPropagation.TOP;
			Object res = PyStrings.search(((StrSearch) operator).getKind().name().toLowerCase(),
					haystack, needle, right.as(PyStrings.Slice.class));
			return new ConstantPropagation(new Constant(
					res instanceof Boolean ? BoolType.INSTANCE : Int32Type.INSTANCE, res, pp.getLocation()));
		}

		if (operator instanceof StringReplace) {
			// python's replace replaces all the occurrences
			if (!left.is(String.class) || !middle.is(String.class) || !right.is(String.class))
				return ConstantPropagation.TOP;
			return string(PyStrings.replace(left.as(String.class), middle.as(String.class),
					right.as(String.class), -1), pp);
		}

		if (operator == StrReplaceCount.INSTANCE) {
			boolean isBytes = left.is(PyBytes.class);
			String t = text(left);
			if (t == null || !middle.is(PyStrings.Pair.class) || !(right.getConstant() instanceof Integer))
				return ConstantPropagation.TOP;
			PyStrings.Pair p = middle.as(PyStrings.Pair.class);
			Class<?> expected = isBytes ? PyBytes.class : String.class;
			if (!expected.isInstance(p.first) || !expected.isInstance(p.second))
				return ConstantPropagation.TOP;
			String old = isBytes ? ((PyBytes) p.first).toLatin1() : (String) p.first;
			String repl = isBytes ? ((PyBytes) p.second).toLatin1() : (String) p.second;
			return text(PyStrings.replace(t, old, repl, (Integer) right.getConstant()), isBytes, pp);
		}

		if (operator instanceof Codec) {
			PyCodecs.Result res = codec((Codec) operator, left, middle, right);
			if (res == null || !res.decided)
				return ConstantPropagation.TOP;
			if (res.exception != null)
				// raised by the caller
				return ConstantPropagation.BOTTOM;
			return res.value instanceof PyBytes ? bytes((PyBytes) res.value, pp) : string((String) res.value, pp);
		}

		return ConstantPropagation.TOP;
	}

	private static ConstantPropagation bytes(
			PyBytes value,
			ProgramPoint pp) {
		return new ConstantPropagation(new Constant(PyBytesType.INSTANCE, value, pp.getLocation()));
	}

	private ConstantPropagation bytesBinary(
			BytesOperation operator,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (!left.is(PyBytes.class))
			return ConstantPropagation.TOP;
		PyBytes b = left.as(PyBytes.class);

		if (operator == BytesOperation.CONCAT) {
			if (!right.is(PyBytes.class))
				return ConstantPropagation.TOP;
			return bytes(PyBytes.fromLatin1(b.toLatin1() + right.as(PyBytes.class).toLatin1()), pp);
		}

		if (operator == BytesOperation.REPEAT) {
			Long n = index(right);
			if (n == null || n * b.length() > 1_000_000)
				return ConstantPropagation.TOP;
			return bytes(PyBytes.fromLatin1(n <= 0 ? "" : b.toLatin1().repeat(n.intValue())), pp);
		}

		if (operator == BytesOperation.GETITEM) {
			Long i = index(right);
			if (i == null)
				return ConstantPropagation.TOP;
			if (i < 0)
				i += b.length();
			if (i < 0 || i >= b.length())
				// IndexError, raised by the caller
				return ConstantPropagation.BOTTOM;
			return new ConstantPropagation(new Constant(Int32Type.INSTANCE, b.get(i.intValue()), pp.getLocation()));
		}

		if (operator == BytesOperation.GETSLICE) {
			if (!right.is(PyStrings.Slice.class))
				return ConstantPropagation.TOP;
			String res = PyStrings.getSlice(b.toLatin1(), right.as(PyStrings.Slice.class));
			// ValueError, raised by the caller
			return res == null ? ConstantPropagation.BOTTOM : bytes(PyBytes.fromLatin1(res), pp);
		}

		// contains
		boolean res;
		if (right.is(PyBytes.class))
			res = b.toLatin1().contains(right.as(PyBytes.class).toLatin1());
		else {
			Long n = index(right);
			if (n == null)
				return ConstantPropagation.TOP;
			if (n < 0 || n > 255)
				// ValueError, raised by the caller
				return ConstantPropagation.BOTTOM;
			res = b.toLatin1().indexOf((char) n.intValue()) >= 0;
		}
		return new ConstantPropagation(new Constant(BoolType.INSTANCE, res, pp.getLocation()));
	}

	// the outcome of a codec, or null if the operands are not known
	private static PyCodecs.Result codec(
			Codec codec,
			ConstantPropagation value,
			ConstantPropagation encoding,
			ConstantPropagation errors) {
		if (value.isTop() || encoding.isTop() || errors.isTop())
			return null;
		String enc = encoding.constant.getStaticType().isNullType() ? "utf-8"
				: encoding.is(String.class) ? encoding.as(String.class) : null;
		String err = errors.constant.getStaticType().isNullType() ? "strict"
				: errors.is(String.class) ? errors.as(String.class) : null;
		if (enc == null || err == null)
			return null;
		if (codec.isEncode())
			return value.is(String.class) ? PyCodecs.encode(value.as(String.class), enc, err) : null;
		return value.is(PyBytes.class) ? PyCodecs.decode(value.as(PyBytes.class), enc, err) : null;
	}

	@Override
	public it.unive.lisa.lattices.Satisfiability satisfiesTernaryExpression(
			TernaryExpression expression,
			ConstantPropagation left,
			ConstantPropagation middle,
			ConstantPropagation right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (expression.getOperator() instanceof CodecRaises) {
			CodecRaises raises = (CodecRaises) expression.getOperator();
			PyCodecs.Result res = codec(raises.getCodec(), left, middle, right);
			if (res == null || !res.decided)
				return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(raises.getException().equals(res.exception));
		}
		return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
	}

	@Override
	public it.unive.lisa.lattices.Satisfiability satisfiesUnaryExpression(
			UnaryExpression expression,
			ConstantPropagation arg,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (expression.getOperator() == FromHexRaises.INSTANCE && !arg.isTop() && arg.is(String.class))
			return it.unive.lisa.lattices.Satisfiability.fromBoolean(PyCodecs.fromHex(arg.as(String.class)) == null);
		return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
	}

	// the base of int(x, base), or null if it is not a valid one
	private static Integer base(
			ConstantPropagation base) {
		if (base.constant.getStaticType().isNullType())
			return 10;
		Long b = index(base);
		return b == null ? null : b.intValue();
	}

	private static ConstantPropagation toInt(
			ConstantPropagation x,
			ConstantPropagation base,
			ProgramPoint pp) {
		if (x.isTop() || base.isTop())
			return ConstantPropagation.TOP;
		Object v = x.getConstant();
		java.math.BigInteger res;
		if (v instanceof String) {
			Integer b = base(base);
			if (b == null)
				return ConstantPropagation.TOP;
			res = PyNumbers.parseInt((String) v, b);
			if (res == null)
				// ValueError, raised by the caller
				return ConstantPropagation.BOTTOM;
		} else if (v instanceof Boolean)
			res = (Boolean) v ? java.math.BigInteger.ONE : java.math.BigInteger.ZERO;
		else if (v instanceof Integer || v instanceof Long)
			res = java.math.BigInteger.valueOf(((Number) v).longValue());
		else if (v instanceof Float || v instanceof Double) {
			double d = ((Number) v).doubleValue();
			if (!Double.isFinite(d))
				return ConstantPropagation.TOP;
			// truncation towards zero
			res = new java.math.BigDecimal(d).toBigInteger();
		} else
			return ConstantPropagation.TOP;
		if (res.bitLength() > 31)
			// ints are tracked as 32-bit integers
			return ConstantPropagation.TOP;
		return new ConstantPropagation(new Constant(Int32Type.INSTANCE, res.intValue(), pp.getLocation()));
	}

	// whether int(x, base) (or float(x)) raises ValueError, or null if unknown
	private static Boolean conversionRaises(
			boolean toInt,
			ConstantPropagation x,
			ConstantPropagation base) {
		Object v = x.getConstant();
		if (v instanceof String) {
			if (!toInt)
				return PyNumbers.parseFloat((String) v) == null;
			Integer b = base(base);
			return b == null ? null : PyNumbers.parseInt((String) v, b) == null;
		}
		if (toInt && (v instanceof Float || v instanceof Double))
			// int(nan) raises ValueError, int(inf) OverflowError (not modeled)
			return Double.isNaN(((Number) v).doubleValue()) ? Boolean.TRUE
					: Double.isInfinite(((Number) v).doubleValue()) ? null : Boolean.FALSE;
		if (v instanceof Number || v instanceof Boolean)
			return false;
		return null;
	}

	// the text of a str or bytes constant (bytes as their latin-1 view), or
	// null
	private static String text(
			ConstantPropagation c) {
		if (c.is(String.class))
			return c.as(String.class);
		if (c.is(PyBytes.class))
			return c.as(PyBytes.class).toLatin1();
		return null;
	}

	// a str, or bytes if the receiver is bytes
	private static ConstantPropagation text(
			String value,
			boolean bytes,
			ProgramPoint pp) {
		return bytes ? bytes(PyBytes.fromLatin1(value), pp) : string(value, pp);
	}

	// the substring searched in a str or bytes: bytes also accept a single
	// byte (an int between 0 and 255)
	private static String needle(
			ConstantPropagation c,
			boolean bytes) {
		if (bytes && !c.is(String.class)) {
			Long n = c.is(PyBytes.class) ? null : index(c);
			if (n != null)
				return n >= 0 && n <= 255 ? String.valueOf((char) n.intValue()) : null;
			return c.is(PyBytes.class) ? c.as(PyBytes.class).toLatin1() : null;
		}
		return !bytes && c.is(String.class) ? c.as(String.class) : null;
	}

	private static ConstantPropagation string(
			String value,
			ProgramPoint pp) {
		return new ConstantPropagation(new Constant(StringType.INSTANCE, value, pp.getLocation()));
	}

	private static Long index(
			ConstantPropagation c) {
		Object v = c.getConstant();
		if (v instanceof Integer || v instanceof Long)
			return ((Number) v).longValue();
		if (v instanceof Boolean)
			return (Boolean) v ? 1L : 0L;
		return null;
	}

	private ConstantPropagation stringBinary(
			BinaryOperator operator,
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop())
			return ConstantPropagation.TOP;
		if (operator instanceof StrStrip && left.is(PyBytes.class)) {
			// bytes strip ascii whitespace, or the given bytes
			StrStrip strip = (StrStrip) operator;
			String chars;
			if (right.constant.getStaticType().isNullType())
				chars = PyCodecs.ASCII_WHITESPACE;
			else if (right.is(PyBytes.class))
				chars = right.as(PyBytes.class).toLatin1();
			else
				return ConstantPropagation.TOP;
			return bytes(PyBytes.fromLatin1(PyStrings.strip(left.as(PyBytes.class).toLatin1(), chars,
					strip.stripsLeft(), strip.stripsRight())), pp);
		}
		if (!left.is(String.class))
			return ConstantPropagation.TOP;
		String s = left.as(String.class);

		if (operator == StrGetItem.INSTANCE) {
			Long i = index(right);
			if (i == null)
				return ConstantPropagation.TOP;
			String res = PyStrings.getItem(s, i);
			// IndexError, raised by the caller
			return res == null ? ConstantPropagation.BOTTOM : string(res, pp);
		}

		if (operator == StrGetSlice.INSTANCE) {
			if (!right.is(PyStrings.Slice.class))
				return ConstantPropagation.TOP;
			String res = PyStrings.getSlice(s, right.as(PyStrings.Slice.class));
			// ValueError, raised by the caller
			return res == null ? ConstantPropagation.BOTTOM : string(res, pp);
		}

		if (operator instanceof StrStrip) {
			StrStrip strip = (StrStrip) operator;
			String chars;
			if (right.constant.getStaticType().isNullType())
				chars = null;
			else if (right.is(String.class))
				chars = right.as(String.class);
			else
				return ConstantPropagation.TOP;
			return string(PyStrings.strip(s, chars, strip.stripsLeft(), strip.stripsRight()), pp);
		}

		// searches without bounds
		if (!right.is(String.class))
			return ConstantPropagation.TOP;
		String sub = right.as(String.class);
		PyStrings.Slice all = new PyStrings.Slice(null, null, null);
		String kind = operator instanceof StringIndexOf ? "find"
				: operator instanceof StringLastIndexOf ? "rfind"
						: operator instanceof StringStartsWith ? "startswith" : "endswith";
		Object res = PyStrings.search(kind, s, sub, all);
		return new ConstantPropagation(new Constant(
				res instanceof Boolean ? BoolType.INSTANCE : Int32Type.INSTANCE, res, pp.getLocation()));
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
		if (operator instanceof ConversionRaises) {
			if (left.isTop() || right.isTop() || left.isBottom() || right.isBottom())
				return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
			Boolean raises = conversionRaises(((ConversionRaises) operator).isInt(), left, right);
			return raises == null ? it.unive.lisa.lattices.Satisfiability.UNKNOWN
					: it.unive.lisa.lattices.Satisfiability.fromBoolean(raises);
		}
		if (operator instanceof StringFormatRaises) {
			if (left.isTop() || right.isTop() || left.isBottom() || right.isBottom() || !left.is(String.class))
				return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
			PyPercentFormat.Result res = PyPercentFormat.format(left.as(String.class), right.getConstant());
			if (!res.decided)
				return it.unive.lisa.lattices.Satisfiability.UNKNOWN;
			return it.unive.lisa.lattices.Satisfiability
					.fromBoolean(((StringFormatRaises) operator).getException().equals(res.exception));
		}
		if (!(operator instanceof it.unive.lisa.symbolic.value.operator.ComparisonOperator)
				|| left.isTop() || right.isTop() || left.isBottom() || right.isBottom())
			return it.unive.lisa.lattices.Satisfiability.UNKNOWN;

		// bool is a subclass of int in python: True == 1
		Object l = left.getConstant() instanceof Boolean ? ((Boolean) left.getConstant() ? 1 : 0) : left.getConstant();
		Object r = right.getConstant() instanceof Boolean ? ((Boolean) right.getConstant() ? 1 : 0)
				: right.getConstant();

		// numeric equality must be checked value-wise (0 == 0.0 is true in
		// Python) rather than via Objects.equals, which is class-sensitive:
		// Integer(0).equals(Float(0.0f)) is false even though they denote
		// the same number, e.g. this matters when the same binary operator
		// resolves to both int.__truediv__ and float.__truediv__ for a
		// plain int/int division (Int32Type.canBeAssignedTo(Float32Type) is
		// true in this codebase's type lattice), so a zero-divisor check
		// comparing against a Float32 zero constant must still recognize an
		// Integer(0) divisor as zero
		if (operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonEq.INSTANCE
				|| operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonNe.INSTANCE) {
			boolean eq = (l instanceof Number && r instanceof Number)
					? ((Number) l).doubleValue() == ((Number) r).doubleValue()
					: java.util.Objects.equals(l, r);
			return it.unive.lisa.lattices.Satisfiability
					.fromBoolean(operator == it.unive.lisa.symbolic.value.operator.binary.ComparisonEq.INSTANCE
							? eq
							: !eq);
		}

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
		if (left.isTop() || right.isTop() || !left.is(String.class))
			return ConstantPropagation.TOP;
		PyPercentFormat.Result res = PyPercentFormat.format(left.as(String.class), right.getConstant());
		if (res.decided && res.exception != null)
			// the exception is raised by the caller
			return ConstantPropagation.BOTTOM;
		if (res.value == null)
			return ConstantPropagation.TOP;
		return new ConstantPropagation(new Constant(StringType.INSTANCE, res.value, pp.getLocation()));
	}

	private Constant div(
			ConstantPropagation left,
			ConstantPropagation right,
			ProgramPoint pp) {
		// python's true division always yields a float, even between ints
		// that divide exactly (6 / 3 == 2.0)
		float l = ((Number) left.getConstant()).floatValue();
		float r = ((Number) right.getConstant()).floatValue();
		return new Constant(Float32Type.INSTANCE, l / r, pp.getLocation());
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
			boolean floatResult,
			ProgramPoint pp) {
		if (left.isTop() || right.isTop() || !(left.getConstant() instanceof Number)
				|| !(right.getConstant() instanceof Number))
			return ConstantPropagation.TOP;

		Number base = (Number) left.getConstant();
		Number exp = (Number) right.getConstant();
		boolean intOperands = isIntegral(base) && isIntegral(exp);
		if (intOperands && exp.longValue() >= 0 && !floatResult) {
			// int ** non-negative int is an int (overflows are not modeled)
			double res = Math.pow(base.doubleValue(), exp.doubleValue());
			if (Math.abs(res) > Integer.MAX_VALUE)
				return ConstantPropagation.TOP;
			return new ConstantPropagation(new Constant(Int32Type.INSTANCE, (int) res, pp.getLocation()));
		}

		double b = base.doubleValue();
		double e = exp.doubleValue();
		if (b == 0 && e < 0)
			// ZeroDivisionError, raised by the caller
			return ConstantPropagation.BOTTOM;
		if (b < 0 && e != Math.rint(e))
			// a negative number raised to a fractional power is complex
			return ConstantPropagation.TOP;
		return new ConstantPropagation(new Constant(Float32Type.INSTANCE, (float) Math.pow(b, e), pp.getLocation()));
	}

	private static boolean isIntegral(
			Number n) {
		return n instanceof Integer || n instanceof Long || n instanceof Short || n instanceof Byte;
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
