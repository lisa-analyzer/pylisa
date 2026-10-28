package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyNot;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.List;

/**
 * Native implementation of Python's {@code all(iterable)} and
 * {@code any(iterable)}, given by the truth values of the elements (see
 * {@link PyNot#truth}): {@code all} is {@code False} if an element is false,
 * and {@code True} if they are all true (also when there are none); {@code any}
 * is {@code True} if an element is true, and {@code False} if they are all
 * false. The elements of a list or tuple of known length are checked one by
 * one; otherwise, the truth value of any element (see
 * {@link Iteration#elements}) is checked, and the iterable might be empty
 * unless it is a list or tuple with some elements.
 */
public abstract class AllAny extends PyNative {

	// whether this is all
	private final boolean all;

	protected AllAny(
			CFG cfg,
			CodeLocation location,
			String construct,
			boolean all,
			Expression[] params) {
		super(cfg, location, construct, params);
		this.all = all;
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression iterable = args[0];
		// the truth value of an element that decides the result (false for
		// all, true for any), and the opposite one
		Satisfiability decisive = all ? Satisfiability.NOT_SATISFIED : Satisfiability.SATISFIED;
		Satisfiability neutral = all ? Satisfiability.SATISFIED : Satisfiability.NOT_SATISFIED;

		List<SymbolicExpression> elements = ListModel.exactElements(analysis, state, iterable, this);
		if (elements != null) {
			boolean decided = false, unknown = false;
			for (SymbolicExpression e : elements) {
				Satisfiability sat = PyNot.truth(analysis, state, e, this);
				decided |= sat == decisive;
				unknown |= sat != decisive && sat != neutral;
			}
			return result(analysis, state, !decided, decided || unknown);
		}

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<
				A> any = Iteration.elements(analysis, state, getSubExpressions()[0], iterable, st, getMetaVariables(),
						raising);
		// e.g. the TypeError raised by iterating an int
		AnalysisState<A> result = any.bottomExecution();
		if (any.getExecutionState().isBottom())
			return raising.moveFrom(result);
		// no elements: the iterable is empty
		boolean allDecisive = !any.getExecutionExpressions().isEmpty(), allNeutral = true;
		for (SymbolicExpression e : any.getExecutionExpressions()) {
			Satisfiability sat = PyNot.truth(analysis, any, e, this);
			allDecisive &= sat == decisive;
			allNeutral &= sat == neutral;
		}
		boolean nonEmpty = nonEmpty(analysis, state, iterable);
		return raising.moveFrom(result.lub(result(analysis, state, !(allDecisive && nonEmpty), !allNeutral)));
	}

	// the result: the value of all (or any) if no element decides it, its
	// negation if some element does
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> result(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			boolean undecided,
			boolean decided)
			throws SemanticException {
		CodeLocation loc = getLocation();
		AnalysisState<A> result = state.bottomExecution();
		if (undecided)
			result = result.lub(compute(analysis, state, new Constant(BoolType.INSTANCE, all, loc)));
		if (decided)
			result = result.lub(compute(analysis, state, new Constant(BoolType.INSTANCE, !all, loc)));
		return result;
	}

	// whether the iterable is a list or tuple that surely has some elements
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean nonEmpty(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression iterable)
			throws SemanticException {
		if (!analysis.getRuntimeTypesOf(state, iterable, this).stream()
				.allMatch(t -> ListModel.isList(t) || Unpacking.isTuple(t)))
			return false;
		ListModel.Shape shape = ListModel.shape(analysis, state, iterable, this);
		return shape.prefix() > 0 || (shape.length() != null && shape.length() > 0);
	}
}
