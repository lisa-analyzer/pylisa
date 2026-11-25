package it.unive.pylisa.libraries.random;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.libraries.RangeBuiltin;
import it.unive.pylisa.libraries.lists.ListModel;
import it.unive.pylisa.libraries.sets.SetModel;
import java.util.Set;

/**
 * Native implementation of {@code random.choice(seq)}: any of the elements of a
 * list, a tuple, a {@code str} or a range (see {@link Iteration#elements}),
 * raising {@code IndexError} if {@code seq} might be empty (that is, unless it
 * is a list or a tuple whose length is known to be positive, or a non-empty
 * {@code str} constant). The other builtin values (e.g. an {@code int} or a
 * set, that cannot be indexed) raise {@code TypeError}; for the other values
 * (e.g. a dictionary, or an object, whose {@code __len__} and
 * {@code __getitem__} are called), the result is unknown, and any exception
 * might be raised (see {@link LibrarySpecificationProvider#UNKNOWN_EXCEPTION}).
 */
public class RandomChoice extends PyNative {

	protected RandomChoice(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "choice", params);
	}

	public static RandomChoice build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RandomChoice(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression seq = args[0];
		Set<Type> types = analysis.getRuntimeTypesOf(state, seq, context, this);
		boolean unknown = types.isEmpty() || types.stream().anyMatch(Type::isUntyped);
		boolean sequence = !unknown && types.stream().anyMatch(RandomChoice::indexable);
		// e.g. an int, or a set, that cannot be indexed
		boolean other = !unknown && types.stream().anyMatch(t -> (PyBinaryDispatch.isBuiltinValueType(t)
				&& !t.isStringType() && !(t instanceof PyBytesType)) || SetModel.isSet(t));
		// e.g. a dictionary, or an object, whose __getitem__ is called
		boolean object = unknown || types.stream().anyMatch(t -> !indexable(t) && !SetModel.isSet(t)
				&& (!PyBinaryDispatch.isBuiltinValueType(t) || t instanceof PyBytesType));

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		if (sequence) {
			Expression[] subs = getSubExpressions();
			result = result.lub(Iteration.elements(analysis, context, state, subs.length > 0 ? subs[0] : null, seq,
					st, getMetaVariables(), raising));
			if (!surelyNonEmpty(analysis, context, state, seq, types))
				raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.INDEX_ERROR);
		}
		if (other)
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.TYPE_ERROR);
		if (object) {
			result = result.lub(unknown(analysis, context, state, null));
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.UNKNOWN_EXCEPTION);
		}
		return raising.moveFrom(result);
	}

	// the values that random.choice indexes: lists, tuples, strs and ranges
	private static boolean indexable(
			Type t) {
		return ListModel.isList(t) || Unpacking.isTuple(t) || t.isStringType() || RangeBuiltin.isRange(t);
	}

	// whether the sequence surely has an element
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean surelyNonEmpty(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression seq,
			Set<Type> types)
			throws SemanticException {
		if (seq instanceof Constant && ((Constant) seq).getValue() instanceof String)
			return !((String) ((Constant) seq).getValue()).isEmpty();
		if (!types.stream().allMatch(t -> ListModel.isList(t) || Unpacking.isTuple(t)))
			return false;
		Integer length = ListModel.length(analysis, context, state, seq, st);
		return (length != null && length > 0)
				|| (types.stream().allMatch(ListModel::isList)
						&& ListModel.nonEmpty(analysis, context, state, seq, st));
	}
}
