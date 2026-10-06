package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyBytesType;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;

/**
 * Base class of native implementations of Python methods and functions. It
 * evaluates {@link #semantics} on each combination of the values of the
 * parameters (e.g. the receiver, for methods), and provides helpers to check
 * the types of the arguments: Python raises {@code TypeError} when an argument
 * has the wrong type.
 */
public abstract class PyNative extends NaryExpression implements PluggableStatement {

	/**
	 * The call this native implements.
	 */
	protected Statement st;

	/**
	 * Builds the native.
	 *
	 * @param cfg       the cfg
	 * @param location  the location
	 * @param construct the name of the method or function
	 * @param params    the parameters
	 */
	protected PyNative(
			CFG cfg,
			CodeLocation location,
			String construct,
			Expression[] params) {
		super(cfg, location, construct, params);
	}

	/**
	 * Yields the call this native implements.
	 *
	 * @return the call
	 */
	public Statement getOriginatingStatement() {
		return st;
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		return combinations(interprocedural.getAnalysis(), state, params, new SymbolicExpression[params.length], 0);
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> combinations(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet[] params,
			SymbolicExpression[] args,
			int pos)
			throws SemanticException {
		if (pos == params.length)
			return semantics(analysis, state, args.clone());
		AnalysisState<A> result = state.bottom();
		for (SymbolicExpression e : params[pos]) {
			args[pos] = e;
			result = result.lub(combinations(analysis, state, params, args, pos + 1));
		}
		return result;
	}

	/**
	 * The semantics on the given arguments.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the arguments have been evaluated
	 * @param args     the arguments
	 *
	 * @return the state after the call
	 *
	 * @throws SemanticException if the analysis fails
	 */
	protected abstract <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException;

	/**
	 * A {@code str}.
	 */
	public static final Predicate<Type> STR = Type::isStringType;

	/**
	 * An {@code int} (or a {@code bool}, that is a subclass of it).
	 */
	public static final Predicate<Type> INT = t -> (t.isNumericType() && t.asNumericType().isIntegral())
			|| t.isBooleanType();

	/**
	 * A {@code bytes}.
	 */
	public static final Predicate<Type> BYTES = t -> t instanceof PyBytesType;

	/**
	 * Whether a receiver can be a {@code str} and/or {@code bytes}, for natives
	 * shared between the two classes: the result contains {@code false} if it
	 * can be a {@code str}, and {@code true} if it can be {@code bytes}.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param self     the receiver
	 *
	 * @return the possible kinds of receiver
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> List<Boolean> textModes(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression self)
			throws SemanticException {
		Satisfiability bytes = hasType(analysis, state, self, BYTES);
		List<Boolean> modes = new ArrayList<>(2);
		if (bytes != Satisfiability.SATISFIED)
			modes.add(false);
		if (bytes != Satisfiability.NOT_SATISFIED)
			modes.add(true);
		return modes;
	}

	/**
	 * {@code None}.
	 */
	public static final Predicate<Type> NONE = Type::isNullType;

	/**
	 * Whether {@code arg} certainly ({@code SATISFIED}), possibly
	 * ({@code UNKNOWN}) or certainly not ({@code NOT_SATISFIED}) has a type
	 * satisfying {@code accepted}.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param arg      the argument
	 * @param accepted the accepted types
	 *
	 * @return whether the type of the argument is accepted
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability hasType(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression arg,
			Predicate<Type> accepted)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, arg, this);
		if (types.isEmpty() || types.stream().anyMatch(Type::isUntyped))
			return Satisfiability.UNKNOWN;
		if (types.stream().allMatch(accepted))
			return Satisfiability.SATISFIED;
		if (types.stream().noneMatch(accepted))
			return Satisfiability.NOT_SATISFIED;
		return Satisfiability.UNKNOWN;
	}

	/**
	 * Joins {@code ifTyped} (when the arguments might have the right types)
	 * with a {@code TypeError} (when they might not).
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param typed    whether the arguments have the right types
	 * @param ifTyped  the result when the arguments have the right types
	 *
	 * @return the joined result
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> typeChecked(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Satisfiability typed,
			AnalysisState<A> ifTyped)
			throws SemanticException {
		AnalysisState<A> result = state.bottom();
		if (typed != Satisfiability.NOT_SATISFIED)
			result = result.lub(ifTyped);
		if (typed != Satisfiability.SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	/**
	 * Raises the given exception.
	 *
	 * @param analysis  the analysis
	 * @param state     the current state
	 * @param exception the name of the exception
	 *
	 * @return the state where the exception has been raised
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> raise(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			String exception)
			throws SemanticException {
		return PyExceptions.raise(analysis, state, getCFG(), getLocation(), this, exception);
	}

	/**
	 * Computes the given value.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param value    the value
	 *
	 * @return the state where the value has been computed
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> compute(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value)
			throws SemanticException {
		return analysis.smallStepSemantics(state, value, st);
	}

	/**
	 * Computes an unknown value of the given type.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param type     the type of the value
	 *
	 * @return the state where the value has been computed
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> unknown(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Type type)
			throws SemanticException {
		return analysis.smallStepSemantics(state, new PushAny(type == null ? Untyped.INSTANCE : type, getLocation()),
				st);
	}
}
