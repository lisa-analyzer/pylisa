package it.unive.pylisa.symbolic.operators.strings;

import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code s.replace(old, new, count)} with an explicit {@code count}:
 * the operands are the string, the pair {@code (old, new)} (built with
 * {@link ArgPair}) and the count, where a negative count replaces all the
 * occurrences.
 */
public class StrReplaceCount implements TernaryOperator {

	/**
	 * The singleton instance of this class.
	 */
	public static final StrReplaceCount INSTANCE = new StrReplaceCount();

	private StrReplaceCount() {
	}

	@Override
	public Set<Type> typeInference(
			TypeSystem types,
			Set<Type> left,
			Set<Type> middle,
			Set<Type> right) {
		return Collections.singleton(StringType.INSTANCE);
	}

	@Override
	public String toString() {
		return "str.replace";
	}
}
