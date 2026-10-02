package it.unive.pylisa.program.language.parameterassignment;

import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Binds the arguments of a call to the formal parameters of its callee, by
 * Python's rules, as {@link PyAssigningStrategy} does when the call is
 * analysed: positional arguments first, then the extra positional arguments
 * to a {@code *args} parameter, the keyword arguments no parameter is named
 * after to a final {@code **kw} parameter, the other keyword arguments to the
 * parameters they name, and defaults to the parameters left. Where Python
 * raises {@code TypeError} (an argument too many, a keyword naming no
 * parameter, a parameter given twice or not at all), there is no binding. The
 * binding looks at the shape of the call only, not at the values of its
 * arguments.
 */
public final class ArgumentBinding {

	private ArgumentBinding() {
	}

	/**
	 * Binds the arguments of a call to the formal parameters of a callee.
	 *
	 * @param formals the formal parameters of the callee
	 * @param actuals the arguments of the call, keyword arguments as
	 *                    {@link NamedParameterExpression}s
	 *
	 * @return for each formal parameter, in order, the positions of the
	 *             arguments bound to it: one for a plain parameter, any number
	 *             for a {@code *args} or {@code **kw} parameter, none for a
	 *             parameter that takes its default; empty when the arguments
	 *             do not match the parameters
	 */
	public static Optional<List<List<Integer>>> bind(
			Parameter[] formals,
			Expression[] actuals) {
		List<List<Integer>> bound = new ArrayList<>();
		for (int i = 0; i < formals.length; i++)
			bound.add(null);
		int named = actuals.length;
		for (int i = 0; i < actuals.length; i++)
			if (actuals[i] instanceof NamedParameterExpression) {
				named = i;
				break;
			}
		// a positional argument after a keyword one
		for (int i = named; i < actuals.length; i++)
			if (!(actuals[i] instanceof NamedParameterExpression))
				return Optional.empty();

		int aPos = 0;
		int fPos = 0;
		for (; aPos < named && fPos < formals.length; aPos++, fPos++) {
			if (formals[fPos] instanceof VarKeywordParameter)
				return Optional.empty();
			// parameters after * or *args take keywords only
			if (formals[fPos] instanceof VarPositionalParameter || formals[fPos] instanceof KeywordOnlyParameter)
				break;
			bound.set(fPos, List.of(aPos));
		}

		if (fPos < formals.length && formals[fPos] instanceof VarPositionalParameter) {
			List<Integer> extra = new ArrayList<>();
			for (; aPos < named; aPos++)
				extra.add(aPos);
			bound.set(fPos, List.copyOf(extra));
			fPos++;
		}
		// positional arguments left over
		if (aPos < named)
			return Optional.empty();

		boolean keywords = formals.length > 0 && formals[formals.length - 1] instanceof VarKeywordParameter;
		List<Integer> unnamed = new ArrayList<>();
		for (; aPos < actuals.length; aPos++) {
			NamedParameterExpression keyword = (NamedParameterExpression) actuals[aPos];
			int formal = indexOf(formals, keyword.getParameterName());
			if (formal < 0 || formals[formal] instanceof VarPositionalParameter
					|| keywords && formal == formals.length - 1) {
				if (!keywords)
					return Optional.empty();
				unnamed.add(aPos);
			} else if (bound.get(formal) != null)
				return Optional.empty();
			else
				bound.set(formal, List.of(aPos));
		}
		if (keywords)
			bound.set(formals.length - 1, List.copyOf(unnamed));

		for (int i = 0; i < formals.length; i++)
			if (bound.get(i) == null) {
				// *args takes no argument when the positional ones stop before
				// it
				if (formals[i].getDefaultValue() == null && !(formals[i] instanceof VarPositionalParameter))
					return Optional.empty();
				bound.set(i, List.of());
			}
		return Optional.of(List.copyOf(bound));
	}

	private static int indexOf(
			Parameter[] formals,
			String name) {
		for (int i = 0; i < formals.length; i++)
			if (formals[i].getName().equals(name))
				return i;
		return -1;
	}
}
