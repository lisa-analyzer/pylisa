package it.unive.pylisa.libraries.natives;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;

/**
 * The base of the Java models of library callables. A model receives the
 * arguments of the call in the order of the formal parameters of the callable
 * (keyword arguments and default values already bound), and describes the
 * effect of the call on a {@link ModelState}. A library specification binds a
 * callable to its model by the model's class name.
 * <p>
 * Subclasses must also provide the static factory
 * {@code build(CFG, CodeLocation, Expression[])} that the library loader uses
 * to instantiate native implementations.
 * </p>
 */
public abstract class LibraryNative extends NaryExpression implements PluggableStatement {

	private Statement originating;

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param name       the name of the modelled callable
	 * @param parameters the arguments of the call
	 */
	protected LibraryNative(
			CFG cfg,
			CodeLocation location,
			String name,
			Expression... parameters) {
		super(cfg, location, name, parameters);
	}

	@Override
	public final void setOriginatingStatement(
			Statement statement) {
		this.originating = statement;
	}

	/**
	 * Yields the statement of the analysed program that performs the call.
	 *
	 * @return the call statement
	 */
	protected final Statement callStatement() {
		return originating != null ? originating : this;
	}

	/**
	 * Yields the location of the call in the analysed program, which is also
	 * the allocation site of the objects the call creates.
	 *
	 * @return the location
	 */
	protected final CodeLocation callSite() {
		return callStatement().getLocation();
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public final <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] arguments,
			StatementStore<A> expressions)
			throws SemanticException {
		// the model runs without the errors and the halt of earlier
		// statements, so that its own outcome can be checked, and they are
		// joined back afterwards
		AnalysisState<A> input = state.removeAllErrors(true);
		// a mark carried from an earlier call applies to this one on the
		// executions where it is set on all of them
		ModelState<A, D> entry = new ModelState<>(interprocedural.getAnalysis(), input, this, callStatement())
				.withCarriedMarks();
		AnalysisState<A> outcome = model(entry, arguments).analysisState();
		if (!entry.isUnreachable() && hasNoContinuation(outcome) && !diverges())
			throw new SemanticException("The model of " + getConstructName() + " at " + callSite()
					+ " has no continuation for a reachable input: no normal result, no error and no halt."
					+ " A model of a callable that may never return must declare it");
		return outcome.lub(state.bottomExecution());
	}

	/**
	 * Yields whether the modelled callable may never return for some input,
	 * as a blocking call does: its model may then leave no continuation for a
	 * reachable input. Readers of the results must not take the absence of a
	 * normal result of such a call as a certain failure. A model declares it
	 * with {@link Diverges} on its class.
	 *
	 * @return {@code true} if the callable may never return
	 */
	public final boolean diverges() {
		return diverges(getClass());
	}

	/**
	 * Yields whether the callable modelled by the given class may never
	 * return for some input.
	 *
	 * @param model the class of the model
	 *
	 * @return {@code true} if the model's class is annotated with
	 *             {@link Diverges}
	 */
	public static boolean diverges(
			Class<?> model) {
		return model.isAnnotationPresent(Diverges.class);
	}

	/**
	 * Yields whether no execution continues from a state in any way: not
	 * normally, not with an error and not by halting.
	 */
	private static <A extends AbstractLattice<A>> boolean hasNoContinuation(
			AnalysisState<A> state) {
		boolean normal = !state.getExecution().isBottom() && !state.getExecutionState().isBottom();
		return !normal && state.getErrors().isBottom() && state.getHalt().isBottom()
				&& state.getSmashedErrors().isBottom();
	}

	/**
	 * Describes the effect of the call.
	 *
	 * @param <A>       the kind of abstract state
	 * @param <D>       the kind of abstract domain
	 * @param state     the state before the call
	 * @param arguments the arguments, in the order of the formal parameters
	 *
	 * @return the state after the call, whose computed values are the result
	 *             of the call
	 *
	 * @throws SemanticException if the effect cannot be computed
	 */
	protected abstract <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException;
}
