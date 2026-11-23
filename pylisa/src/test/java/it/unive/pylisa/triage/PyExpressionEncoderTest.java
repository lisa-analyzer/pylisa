package it.unive.pylisa.triage;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeTrue;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Float32Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingAdd;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingSub;
import it.unive.lisa.triage.encoding.SortPolicy;
import it.unive.lisa.triage.encoding.SymbolTable;
import it.unive.lisa.triage.encoding.ValueSort;
import it.unive.lisa.triage.smt.FloatingPoints;
import it.unive.lisa.triage.smt.SmtLibProcessSolver;
import it.unive.lisa.triage.smt.SmtResult;
import it.unive.lisa.triage.smt.SmtSolver;
import it.unive.lisa.triage.smt.Sort;
import it.unive.lisa.triage.smt.Term;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.FloorDivision;
import it.unive.pylisa.symbolic.operators.LogicalNot;
import it.unive.pylisa.symbolic.operators.Modulo;
import it.unive.pylisa.symbolic.operators.Power;
import it.unive.pylisa.symbolic.operators.StringConstructor;
import it.unive.pylisa.symbolic.operators.StringMult;
import it.unive.pylisa.symbolic.operators.TrueDivision;
import it.unive.pylisa.symbolic.operators.compare.Is;
import it.unive.pylisa.symbolic.operators.compare.IsInstance;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonGt;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLe;
import it.unive.pylisa.symbolic.operators.compare.PyComparisonLt;
import it.unive.pylisa.symbolic.operators.conversions.Floor;
import it.unive.pylisa.symbolic.operators.conversions.ToFloat;
import it.unive.pylisa.symbolic.operators.conversions.ToInt;
import it.unive.pylisa.symbolic.operators.conversions.ToRepr;
import it.unive.pylisa.symbolic.operators.strings.StrGetItem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Checks the encodings of {@link PyExpressionEncoder} against Python: for each
 * operation and each tested combination of operands, the encoding must denote
 * the value computed by CPython (the expected values below have been computed
 * with it), both when the operands are encoded with the sorts of their kinds
 * (e.g., integers) and when they are values of unknown kind. Tests are skipped
 * if z3 is not available.
 */
public class PyExpressionEncoderTest {

	private static final SourceCodeLocation LOC = new SourceCodeLocation("test", 0, 0);

	private static SmtSolver solver;

	private static int counter;

	/**
	 * A symbol table recording the conditions under which the encoding is
	 * precise, and the requirements of the encoding.
	 */
	private static class Symbols
			implements
			SymbolTable {

		private final List<Term> guards = new ArrayList<>();

		private final List<Term> requirements = new ArrayList<>();

		private boolean havoc;

		private final java.util.Map<Identifier, Term> constants = new HashMap<>();

		/**
		 * The unknown functions applied, with the sorts of their arguments.
		 */
		private final Set<String> applied = new HashSet<>();

		@Override
		public Term constant(
				Identifier id,
				Sort sort) {
			return constants.computeIfAbsent(id, i -> solver.declare(i.getName() + "#" + (counter++), sort));
		}

		@Override
		public Term fresh(
				Sort sort,
				SymbolicExpression origin) {
			havoc = true;
			return solver.declare("fresh#" + (counter++), sort);
		}

		@Override
		public Term fresh(
				Sort sort,
				SymbolicExpression origin,
				Term guard) {
			guards.add(guard);
			return solver.declare("fresh#" + (counter++), sort);
		}

		@Override
		public Term apply(
				String function,
				Sort result,
				List<Term> arguments,
				SymbolicExpression origin,
				Term guard) {
			List<Sort> sorts = new ArrayList<>();
			for (Term arg : arguments)
				sorts.add(arg.getSort());
			applied.add(function + sorts);
			return fresh(result, origin, guard);
		}

		@Override
		public Term evaluable(
				String function,
				Sort result,
				List<Term> arguments,
				java.util.function.Function<List<Number>, Number> evaluator,
				SymbolicExpression origin) {
			// an unconstrained value: if the result depended on it, it would
			// not be the expected one (the functions are tested separately,
			// see floatFunctionsAgreeWithPython)
			return solver.declare("evaluable#" + (counter++), result);
		}

		@Override
		public void exactOnlyIf(
				Term guard) {
			guards.add(guard);
		}

		@Override
		public void require(
				Term condition) {
			requirements.add(condition);
		}
	}

	@BeforeClass
	public static void start() {
		assumeTrue("z3 is not available", SmtLibProcessSolver.isZ3Available());
		solver = SmtLibProcessSolver.z3();
		solver.declareDatatype(ValueSort.DATATYPE);
	}

	@AfterClass
	public static void stop() {
		if (solver != null)
			solver.close();
	}

	private static SymbolicExpression constant(
			Object value) {
		if (value == null)
			return new PyNoneConstant(LOC);
		Type type;
		if (value instanceof Integer || value instanceof Long)
			type = Int32Type.INSTANCE;
		else if (value instanceof Boolean)
			type = BoolType.INSTANCE;
		else if (value instanceof Double || value instanceof Float)
			type = Float32Type.INSTANCE;
		else
			type = StringType.INSTANCE;
		return new Constant(type, value, LOC);
	}

	private static Term term(
			Object value) {
		if (value == null)
			return ValueSort.nullValue();
		if (value instanceof Boolean)
			return Term.bool((Boolean) value);
		if (value instanceof String)
			return Term.string((String) value);
		if (value instanceof Double)
			return FloatingPoints.literal((Double) value);
		return Term.integer(((Number) value).longValue());
	}

	/**
	 * Builds the expression of the given operation over the given operands.
	 */
	private static SymbolicExpression expression(
			String op,
			SymbolicExpression a,
			SymbolicExpression b) {
		Type t = Untyped.INSTANCE;
		switch (op) {
		case "truediv":
			return new BinaryExpression(Float32Type.INSTANCE, a, b, TrueDivision.INSTANCE, LOC);
		case "floordiv":
			return new BinaryExpression(t, a, b, FloorDivision.INSTANCE, LOC);
		case "mod":
			return new BinaryExpression(t, a, b, Modulo.INSTANCE, LOC);
		case "add":
			return new BinaryExpression(t, a, b, NumericNonOverflowingAdd.INSTANCE, LOC);
		case "sub":
			return new BinaryExpression(t, a, b, NumericNonOverflowingSub.INSTANCE, LOC);
		case "mul":
			return new BinaryExpression(t, a, b, NumericNonOverflowingMul.INSTANCE, LOC);
		case "pow2":
			return new BinaryExpression(t, a, constant(2), Power.INSTANCE, LOC);
		case "lt":
			return new BinaryExpression(BoolType.INSTANCE, a, b, PyComparisonLt.INSTANCE, LOC);
		case "le":
			return new BinaryExpression(BoolType.INSTANCE, a, b, PyComparisonLe.INSTANCE, LOC);
		case "gt":
			return new BinaryExpression(BoolType.INSTANCE, a, b, PyComparisonGt.INSTANCE, LOC);
		case "ge":
			return new BinaryExpression(BoolType.INSTANCE, a, b, PyComparisonGe.INSTANCE, LOC);
		case "eq":
			return new BinaryExpression(BoolType.INSTANCE, a, b, ComparisonEq.INSTANCE, LOC);
		case "ne":
			return new BinaryExpression(BoolType.INSTANCE, a, b, ComparisonNe.INSTANCE, LOC);
		case "is":
			return new BinaryExpression(BoolType.INSTANCE, a, b, Is.INSTANCE, LOC);
		case "not":
			return new UnaryExpression(BoolType.INSTANCE, a, LogicalNot.INSTANCE, LOC);
		case "int":
			return new BinaryExpression(Int32Type.INSTANCE, a, new PyNoneConstant(LOC), ToInt.INSTANCE, LOC);
		case "float":
			return new UnaryExpression(Float32Type.INSTANCE, a, ToFloat.INSTANCE, LOC);
		case "floor":
			return new UnaryExpression(Int32Type.INSTANCE, a, Floor.INSTANCE, LOC);
		case "str":
			return new UnaryExpression(StringType.INSTANCE, a, StringConstructor.INSTANCE, LOC);
		case "repr":
			return new UnaryExpression(StringType.INSTANCE, a, ToRepr.INSTANCE, LOC);
		case "getitem":
			return new BinaryExpression(StringType.INSTANCE, a, b, StrGetItem.INSTANCE, LOC);
		case "strmul":
			return new BinaryExpression(StringType.INSTANCE, a, b, StringMult.INSTANCE, LOC);
		default:
			throw new IllegalArgumentException(op);
		}
	}

	/**
	 * Checks the encoding of the given operation over the given operands,
	 * yielding whether the encoding is precise for them. If {@code values}, the
	 * operands are identifiers of unknown kind, holding the given values.
	 */
	private static boolean check(
			String op,
			Object a,
			Object b,
			Object expected,
			boolean values)
			throws SemanticException {
		Symbols symbols = new Symbols();
		PyExpressionEncoder encoder = new PyExpressionEncoder(new SortPolicy(new HashMap<>()).withDoubleFloats(),
				symbols);
		List<Term> bindings = new ArrayList<>();
		SymbolicExpression ea, eb;
		if (values) {
			Variable va = new Variable(Untyped.INSTANCE, "a", LOC);
			Variable vb = new Variable(Untyped.INSTANCE, "b", LOC);
			Term ta = va.accept(encoder);
			Term tb = vb.accept(encoder);
			bindings.add(Term.eq(ta, ValueSort.wrap(term(a))));
			bindings.add(Term.eq(tb, ValueSort.wrap(term(b))));
			ea = va;
			eb = vb;
		} else {
			ea = constant(a);
			eb = constant(b);
		}
		Term term = expression(op, ea, eb).accept(encoder);
		if (symbols.havoc)
			return false;
		Term exp = term(expected);
		if (term.getSort().equals(ValueSort.SORT) && !exp.getSort().equals(ValueSort.SORT))
			exp = ValueSort.wrap(exp);
		solver.push();
		for (Term t : bindings)
			solver.assertTerm(t);
		for (Term r : symbols.requirements)
			solver.assertTerm(r);
		for (Term g : symbols.guards)
			solver.assertTerm(g);
		SmtResult precise = solver.check();
		if (precise == SmtResult.SAT) {
			solver.assertTerm(Term.not(Term.eq(term, exp)));
			SmtResult res = solver.check();
			solver.pop();
			assertEquals(op + "(" + a + ", " + b + ") should be " + expected + (values ? " (values)" : ""),
					SmtResult.UNSAT, res);
			return true;
		}
		solver.pop();
		return false;
	}

	// computed with CPython
	private static final Object[][] CASES = {
			{ "truediv", 7, 2, 3.5d },
			{ "truediv", -7, 2, -3.5d },
			{ "truediv", 1, 3, 0.3333333333333333d },
			{ "truediv", 7.5d, 2, 3.75d },
			{ "truediv", 6, 3, 2.0d },
			{ "truediv", 2, 0.5d, 4.0d },
			{ "floordiv", 7, 2, 3 },
			{ "floordiv", -7, 2, -4 },
			{ "floordiv", 7, -2, -4 },
			{ "floordiv", -7, -2, 3 },
			{ "floordiv", 0, 5, 0 },
			{ "mod", -7, 3, 2 },
			{ "mod", 7, -3, -2 },
			{ "mod", 7, 3, 1 },
			{ "mod", -7, -3, -1 },
			{ "mod", 0, -3, 0 },
			{ "add", 1, 2.5d, 3.5d },
			{ "sub", 2.5d, 1, 1.5d },
			{ "mul", 3, 0.5d, 1.5d },
			{ "add", 2, 3, 5 },
			{ "pow2", 3, null, 9 },
			{ "pow2", -2, null, 4 },
			{ "lt", "a", "b", true },
			{ "lt", "abc", "abd", true },
			{ "lt", "Z", "a", true },
			{ "lt", 1, 2.5d, true },
			{ "le", 2.5d, 2, false },
			{ "gt", 3, 2, true },
			{ "ge", "b", "b", true },
			{ "lt", 2, 1, false },
			{ "eq", true, 1, true },
			{ "eq", 1, 1.0d, true },
			{ "eq", "1", 1, false },
			{ "eq", null, null, true },
			{ "eq", null, 0, false },
			{ "eq", "a", "a", true },
			{ "eq", 2, 3, false },
			{ "ne", true, 0, true },
			{ "eq", 0.5d, 0.5d, true },
			{ "not", 0, null, true },
			{ "not", "", null, true },
			{ "not", "a", null, false },
			{ "not", null, null, true },
			{ "not", 0.0d, null, true },
			{ "not", 2.5d, null, false },
			{ "not", true, null, false },
			{ "not", -3, null, false },
			{ "int", -3.7d, null, -3 },
			{ "int", "-42", null, -42 },
			{ "int", "+7", null, 7 },
			{ "int", "007", null, 7 },
			{ "int", true, null, 1 },
			{ "int", 5, null, 5 },
			{ "int", 20000000000.0d, null, 20000000000L },
			{ "floor", -3.5d, null, -4 },
			{ "floor", 2.0d, null, 2 },
			{ "floor", 7, null, 7 },
			{ "str", -12, null, "-12" },
			{ "str", true, null, "True" },
			{ "str", null, null, "None" },
			{ "str", "x", null, "x" },
			{ "repr", 5, null, "5" },
			{ "repr", false, null, "False" },
			{ "getitem", "hello", -1, "o" },
			{ "getitem", "hello", 1, "e" },
			{ "getitem", "hello", -5, "h" },
			{ "strmul", "ab", 3, "ababab" },
			{ "strmul", "ab", 0, "" },
			{ "is", null, null, true },
			{ "is", 0, null, false },
			{ "float", 3, null, 3.0d },
			{ "float", true, null, 1.0d },
			{ "float", -2.5d, null, -2.5d },
	};

	@Test
	public void operationsOnOperandsOfKnownKinds() throws SemanticException {
		for (Object[] c : CASES)
			assertTrue(c[0] + "(" + c[1] + ", " + c[2] + ") is not precise",
					check((String) c[0], c[1], c[2], c[3], false));
	}

	@Test
	public void operationsOnValuesOfUnknownKinds() throws SemanticException {
		for (Object[] c : CASES)
			// the repetition is precise only for constant counts
			if (!c[0].equals("strmul") || c[3].equals(""))
				assertTrue(c[0] + "(" + c[1] + ", " + c[2] + ") is not precise on values",
						check((String) c[0], c[1], c[2], c[3], true));
	}

	@Test
	public void imprecisionIsNeverWrong() throws SemanticException {
		// operations that are not encoded precisely are never wrong
		check("int", " 5", null, 5, false);
		check("int", "1_000", null, 1000, false);
		check("truediv", 9007199254740993L, 1, 9007199254740992.0d, false);
		check("repr", "x", null, "'x'", false);
		check("eq", 9007199254740993L, 9007199254740992.0d, false, false);
	}

	/**
	 * Yields the unknown functions applied by the encoding of the given
	 * operation over values of unknown kinds.
	 */
	private static Set<String> applied(
			SymbolicExpression expression)
			throws SemanticException {
		Symbols symbols = new Symbols();
		PyExpressionEncoder encoder = new PyExpressionEncoder(new SortPolicy(new HashMap<>()).withDoubleFloats(),
				symbols);
		expression.accept(encoder);
		return symbols.applied;
	}

	@Test
	public void unknownFunctionsAreSpecificToTheOperation() throws SemanticException {
		// unknown functions are identified by their name and sorts: different
		// operations over the same operands must not be forced to be equal
		Variable a = new Variable(Untyped.INSTANCE, "a", LOC);
		Variable b = new Variable(Untyped.INSTANCE, "b", LOC);
		Set<String> seen = new HashSet<>();
		for (String op : List.of("add", "sub", "mul", "floordiv", "mod", "truediv"))
			for (String function : applied(expression(op, a, b)))
				assertTrue(op + " shares the unknown function " + function, seen.add(function));
		// the class of isinstance and the base of int are arguments
		Variable c = new Variable(Untyped.INSTANCE, "c", LOC);
		for (BinaryExpression e : List.of(
				new BinaryExpression(BoolType.INSTANCE, a, c, IsInstance.INSTANCE, LOC),
				new BinaryExpression(Int32Type.INSTANCE, a, c, ToInt.INSTANCE, LOC))) {
			Set<String> functions = applied(e);
			assertEquals(e + " applies " + functions, 1, functions.size());
			String function = functions.iterator().next();
			assertTrue(e + " applies " + function, function.endsWith("[" + ValueSort.SORT + ", " + ValueSort.SORT + "]"));
		}
	}

	@Test
	public void floatLiteralsAreTheirDecimalValues() throws SemanticException {
		// 0.1 is parsed as a 32-bits number, but it is Python's 0.1
		Symbols symbols = new Symbols();
		PyExpressionEncoder encoder = new PyExpressionEncoder(new SortPolicy(new HashMap<>()).withDoubleFloats(),
				symbols);
		Term term = new Constant(Float32Type.INSTANCE, 0.1f, LOC).accept(encoder);
		assertEquals(FloatingPoints.literal(0.1d), term);
	}

	// computed with CPython: a, b, a // b, a % b
	private static final double[][] FLOAT_DIVISIONS = {
			{ 7.5d, 2.0d, 3.0d, 1.5d },
			{ -7.5d, 2.0d, -4.0d, 0.5d },
			{ 1.0d, 0.1d, 9.0d, 0.09999999999999995d },
			{ -0.0d, 1.0d, -0.0d, 0.0d },
			{ 5.0d, -3.0d, -2.0d, -1.0d },
			{ 7.0d, 2.5d, 2.0d, 2.0d },
			{ -1.0d, Double.POSITIVE_INFINITY, -1.0d, Double.POSITIVE_INFINITY },
			{ 1e+308d, 1e-308d, Double.POSITIVE_INFINITY, 3.498445546245627e-309d },
	};

	// computed with CPython: a, b, a ** b
	private static final double[][] FLOAT_POWERS = {
			{ 2.0d, 0.5d, 1.4142135623730951d },
			{ 2.0d, -1.0d, 0.5d },
			{ -2.0d, 3.0d, -8.0d },
			{ 1.0d, Double.NaN, 1.0d },
			{ 10.0d, -2.0d, 0.01d },
			{ 0.0d, 0.0d, 1.0d },
	};

	@Test
	public void floatFunctionsAgreeWithPython() {
		for (double[] c : FLOAT_DIVISIONS) {
			assertEquals(c[0] + " // " + c[1], Double.doubleToLongBits(c[2]),
					Double.doubleToLongBits(PyExpressionEncoder.floorDiv(c[0], c[1])));
			assertEquals(c[0] + " % " + c[1], Double.doubleToLongBits(c[3]),
					Double.doubleToLongBits(PyExpressionEncoder.mod(c[0], c[1])));
		}
		for (double[] c : FLOAT_POWERS)
			assertEquals(c[0] + " ** " + c[1], Double.doubleToLongBits(c[2]),
					Double.doubleToLongBits(PyExpressionEncoder.pow(c[0], c[1])));
	}
}
