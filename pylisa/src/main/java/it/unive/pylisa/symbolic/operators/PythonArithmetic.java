package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.program.type.Float64Type;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingAdd;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingSub;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingMul;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingDiv;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingRem;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.HashSet;
import java.util.Set;

/**
 * The arithmetic operators of Python on numbers. They are LiSA's arithmetic
 * operators, except that booleans are numbers: in type inference a boolean
 * operand counts as an integer, as {@code True + 1} is the integer {@code 2}.
 */
public final class PythonArithmetic {

	private PythonArithmetic() {
	}

	/**
	 * Yields the types of an operand with booleans seen as integers.
	 *
	 * @param types the types of the operand
	 *
	 * @return the types, where the boolean type is replaced by the integer
	 *             type
	 */
	public static Set<Type> asNumbers(
			Set<Type> types) {
		if (types.stream().noneMatch(Type::isBooleanType))
			return types;
		Set<Type> result = new HashSet<>();
		for (Type type : types)
			result.add(type.isBooleanType() ? Int32Type.INSTANCE : type);
		return result;
	}

	/**
	 * Python's {@code +} on numbers.
	 */
	public static final class Add extends NumericNonOverflowingAdd {

		/**
		 * The singleton instance of this class.
		 */
		public static final Add INSTANCE = new Add();

		private Add() {
		}

		@Override
		public Set<Type> typeInference(
				TypeSystem types,
				Set<Type> left,
				Set<Type> right) {
			return super.typeInference(types, asNumbers(left), asNumbers(right));
		}
	}

	/**
	 * Python's {@code -} on numbers.
	 */
	public static final class Sub extends NumericNonOverflowingSub {

		/**
		 * The singleton instance of this class.
		 */
		public static final Sub INSTANCE = new Sub();

		private Sub() {
		}

		@Override
		public Set<Type> typeInference(
				TypeSystem types,
				Set<Type> left,
				Set<Type> right) {
			return super.typeInference(types, asNumbers(left), asNumbers(right));
		}
	}

	/**
	 * Python's {@code *} on numbers.
	 */
	public static final class Mul extends NumericNonOverflowingMul {

		/**
		 * The singleton instance of this class.
		 */
		public static final Mul INSTANCE = new Mul();

		private Mul() {
		}

		@Override
		public Set<Type> typeInference(
				TypeSystem types,
				Set<Type> left,
				Set<Type> right) {
			return super.typeInference(types, asNumbers(left), asNumbers(right));
		}
	}

	/**
	 * Python's {@code /} on numbers.
	 */
	public static final class Div extends NumericNonOverflowingDiv {

		/**
		 * The singleton instance of this class.
		 */
		public static final Div INSTANCE = new Div();

		private Div() {
		}

		@Override
		public Set<Type> typeInference(
				TypeSystem types,
				Set<Type> left,
				Set<Type> right) {
			// true division always yields a float
			return super.typeInference(types, asNumbers(left), asNumbers(right)).isEmpty() ? Set.of()
					: Set.of(Float64Type.INSTANCE);
		}
	}

	/**
	 * Python's {@code %} on numbers.
	 */
	public static final class Rem extends NumericNonOverflowingRem {

		/**
		 * The singleton instance of this class.
		 */
		public static final Rem INSTANCE = new Rem();

		private Rem() {
		}

		@Override
		public Set<Type> typeInference(
				TypeSystem types,
				Set<Type> left,
				Set<Type> right) {
			return super.typeInference(types, asNumbers(left), asNumbers(right));
		}
	}
}
