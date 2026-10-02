package it.unive.pylisa.checks;

import it.unive.lisa.lattices.Satisfiability;

/**
 * What the analysis established about an {@code assert} statement, over every
 * execution that reaches it.
 */
public enum AssertionVerdict {

	/**
	 * The condition holds in every execution that reaches the assertion.
	 */
	PROVED,

	/**
	 * The condition is false in every execution that reaches the assertion.
	 */
	FAILS,

	/**
	 * The analysis could not decide the condition: it may hold in some
	 * executions and fail in others, or be too imprecise to tell.
	 */
	MAY_FAIL,

	/**
	 * No execution reaches the assertion, so nothing was checked.
	 */
	UNREACHABLE,

	/**
	 * The function holding the assertion was not analysed at all (for
	 * instance, a callback that no analysed code runs), so nothing is known
	 * about it.
	 */
	NOT_ANALYSED;

	/**
	 * Yields the verdict for one analysis context from the satisfiability of
	 * the condition in that context.
	 *
	 * @param satisfiability the satisfiability of the condition
	 *
	 * @return the verdict
	 */
	static AssertionVerdict of(
			Satisfiability satisfiability) {
		switch (satisfiability) {
		case SATISFIED:
			return PROVED;
		case NOT_SATISFIED:
			return FAILS;
		case BOTTOM:
			// a reachable state in which the condition has no value: the
			// domains could not evaluate it, which decides nothing
			return MAY_FAIL;
		default:
			return MAY_FAIL;
		}
	}

	/**
	 * Yields the verdict once the analysis may have misrepresented some
	 * executions (because the frontend translated part of the program
	 * unsoundly): only the verdict that decides nothing survives.
	 *
	 * @return the verdict
	 */
	AssertionVerdict unreliable() {
		return this == NOT_ANALYSED ? NOT_ANALYSED : MAY_FAIL;
	}

	/**
	 * Combines the verdicts of two sets of executions reaching the same
	 * assertion, such as two analysis contexts: the assertion is proved (or
	 * fails) only if it is proved (or fails) in both, and executions that do
	 * not reach it do not count.
	 *
	 * @param other the verdict of the other executions
	 *
	 * @return the combined verdict
	 */
	AssertionVerdict combine(
			AssertionVerdict other) {
		if (this == NOT_ANALYSED || other == NOT_ANALYSED)
			return this == other ? NOT_ANALYSED : MAY_FAIL;
		if (this == UNREACHABLE)
			return other;
		if (other == UNREACHABLE || other == this)
			return this;
		return MAY_FAIL;
	}
}
