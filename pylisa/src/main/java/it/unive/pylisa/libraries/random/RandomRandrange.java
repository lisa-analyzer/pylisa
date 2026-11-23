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
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalAnd;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import java.util.Set;

/**
 * Native implementation of {@code random.randrange(start, stop=None, step=1)}:
 * an {@code int} of {@code range(start, stop, step)} (of {@code range(start)}
 * without {@code stop}), raising {@code ValueError} if the range might be
 * empty. With the default {@code step}, the result is known to be in
 * {@code [0, start)} (or {@code [start, stop)}), and it is related to the
 * bounds (e.g. {@code randrange(len(xs))} is a valid index of {@code xs}); with
 * another {@code step}, it is any {@code int}, and {@code ValueError} is also
 * raised for a {@code step} of {@code 0}. Arguments that are builtin values
 * other than {@code int}s (e.g. a {@code float}) raise {@code TypeError}; for
 * objects, the result is unknown, as their class might define
 * {@code __index__}.
 */
public class RandomRandrange extends PyNative {

	protected RandomRandrange(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "randrange", params);
	}

	public static RandomRandrange build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new RandomRandrange(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		boolean object = false;
		for (SymbolicExpression arg : args) {
			if (missing(arg))
				continue;
			Set<Type> types = analysis.getRuntimeTypesOf(state, arg, context, this);
			boolean unknown = types.isEmpty() || types.stream().anyMatch(Type::isUntyped);
			if (unknown || types.stream().anyMatch(t -> !INT.test(t) && PyBinaryDispatch.isBuiltinValueType(t)))
				raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.TYPE_ERROR);
			if (unknown || types.stream().anyMatch(t -> !PyBinaryDispatch.isBuiltinValueType(t)))
				object = true;
		}

		AnalysisState<A> result = state.bottomExecution();
		if (mightBeInts(analysis, context, state, args)) {
			Constant zero = new Constant(Int32Type.INSTANCE, 0, loc);
			boolean unitStep = missing(args[2]) || (args[2] instanceof Constant
					&& Integer.valueOf(1).equals(((Constant) args[2]).getValue()));
			Variable element = new Variable(Int32Type.INSTANCE, "$randrange@" + loc.getCodeLocation(), loc);
			AnalysisState<A> chosen = analysis.assign(state, element, new PushAny(Int32Type.INSTANCE, loc), context,
					this);
			if (unitStep) {
				SymbolicExpression low = missing(args[1]) ? zero : args[0], high = missing(args[1]) ? args[0] : args[1];
				BinaryExpression nonEmpty = new BinaryExpression(BoolType.INSTANCE, low, high, ComparisonLt.INSTANCE,
						loc);
				Satisfiability sat = analysis.satisfies(state, nonEmpty, context, this);
				if (sat != Satisfiability.SATISFIED)
					raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.VALUE_ERROR);
				if (sat != Satisfiability.NOT_SATISFIED)
					// low <= element < high
					result = analysis.assume(chosen, new BinaryExpression(BoolType.INSTANCE,
							new BinaryExpression(BoolType.INSTANCE, low, element, ComparisonLe.INSTANCE, loc),
							new BinaryExpression(BoolType.INSTANCE, element, high, ComparisonLt.INSTANCE, loc),
							LogicalAnd.INSTANCE, loc), context, this, this);
			} else {
				raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.VALUE_ERROR);
				result = chosen;
			}
			if (!result.getExecutionState().isBottom()) {
				getMetaVariables().add(element);
				result = result.withExecutionExpression(element);
			}
		}
		if (object)
			result = result.lub(unknown(analysis, context, state, null));
		return raising.moveFrom(result);
	}

	// whether each of the arguments might be an int
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean mightBeInts(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		for (SymbolicExpression arg : args)
			if (!missing(arg)) {
				Set<Type> types = analysis.getRuntimeTypesOf(state, arg, context, this);
				if (!types.isEmpty() && types.stream().noneMatch(t -> INT.test(t) || t.isUntyped()))
					return false;
			}
		return true;
	}
}
