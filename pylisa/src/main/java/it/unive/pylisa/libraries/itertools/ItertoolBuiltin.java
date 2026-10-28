package it.unive.pylisa.libraries.itertools;

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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import java.util.function.Predicate;

/**
 * The functions of {@code itertools} creating iterators (see
 * {@link ItertoolsModel}).
 */
abstract class ItertoolBuiltin extends PyNative {

	/**
	 * The numbers (also {@code bool}s).
	 */
	static final Predicate<Type> NUMBER = t -> t.isNumericType() || t.isBooleanType();

	protected ItertoolBuiltin(
			CFG cfg,
			CodeLocation location,
			String name,
			Expression[] params) {
		super(cfg, location, name, params);
	}

	// the iterator of the given class, if the arguments might be of the
	// right types, and TypeError if they might not
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> checked(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Satisfiability typed,
			String cls,
			String[] names,
			SymbolicExpression[] children)
			throws SemanticException {
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		return raising.moveFrom(checked(analysis, state, typed, cls, names, children, raising));
	}

	// as checked, where the TypeError is collected by raising
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> checked(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Satisfiability typed,
			String cls,
			String[] names,
			SymbolicExpression[] children,
			PyExceptions.Raising<A, D> raising)
			throws SemanticException {
		AnalysisState<A> created = state.bottomExecution();
		if (typed != Satisfiability.NOT_SATISFIED)
			created = ItertoolsModel.create(analysis, state, st, cls, names, children, getMetaVariables());
		return typeChecked(state, typed, created, raising);
	}

	// ValueError, collected by raising, if the value might be negative
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> void negative(
			Analysis<A, D> analysis,
			PyExceptions.Raising<A, D> raising,
			AnalysisState<A> state,
			SymbolicExpression value)
			throws SemanticException {
		CodeLocation loc = getLocation();
		Satisfiability negative = analysis.satisfies(state, new BinaryExpression(BoolType.INSTANCE, value,
				new Constant(Int32Type.INSTANCE, 0, loc), ComparisonLt.INSTANCE, loc), st);
		if (negative != Satisfiability.NOT_SATISFIED)
			raise(raising, state, LibrarySpecificationProvider.VALUE_ERROR);
	}

	// permutations and combinations (where r might be None, if noneR)
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> tuples(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args,
			String cls,
			boolean noneR)
			throws SemanticException {
		SymbolicExpression iterable = args[0], r = args[1];
		boolean none = noneR && hasType(analysis, state, r, NONE) == Satisfiability.SATISFIED;
		Satisfiability typed = Iteration.iterable(analysis, state, iterable, st);
		if (!none)
			typed = typed.and(noneR ? hasType(analysis, state, r, t -> INT.test(t) || NONE.test(t))
					: hasType(analysis, state, r, INT));
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
		AnalysisState<A> result = checked(analysis, state, typed, cls,
				new String[] { ItertoolsModel.ITERABLE, ItertoolsModel.R }, new SymbolicExpression[] { iterable, r },
				raising);
		if (!none)
			negative(analysis, raising, state, r);
		return raising.moveFrom(result);
	}
}
