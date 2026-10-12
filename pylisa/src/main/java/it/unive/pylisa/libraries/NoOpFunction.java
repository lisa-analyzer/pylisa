package it.unive.pylisa.libraries;

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
import it.unive.lisa.program.cfg.statement.NoOp;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;

/**
 * A function that has no effect on the analysis state (e.g., {@code print}, see
 * {@code stdlib.txt}). Its arguments are still evaluated, and their values are
 * then discarded: the meta variables of the arguments (e.g., the return values
 * of calls) are forgotten.
 */
public class NoOpFunction extends NaryExpression implements PluggableStatement {

	/**
	 * The statement this function originates from.
	 */
	protected Statement st;

	/**
	 * Builds the function.
	 *
	 * @param cfg           the cfg where the function is called
	 * @param location      the location of the call
	 * @param constructName the name of the construct
	 * @param parameters    the arguments of the call
	 */
	protected NoOpFunction(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression... parameters) {
		super(cfg, location, constructName, parameters);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	/**
	 * Builds the function. This is the factory method used for native functions
	 * defined in the library specification files.
	 *
	 * @param cfg      the cfg where the function is called
	 * @param location the location of the call
	 * @param exprs    the arguments of the call
	 *
	 * @return the function
	 */
	public static NoOpFunction build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NoOpFunction(cfg, location, "noop-func", exprs);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = new NoOp(getCFG(), getLocation()).forwardSemantics(state, interprocedural,
				expressions);
		// the values of the arguments are consumed here: their meta variables
		// are not needed anymore
		for (Expression sub : getSubExpressions())
			result = result.forgetIdentifiers(sub.getMetaVariables(), this);
		return result;
	}
}