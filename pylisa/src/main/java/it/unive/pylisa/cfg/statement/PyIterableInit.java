package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;

/**
 * The assignment of the iterable of a {@code for} loop to the variable that
 * only the loop reads. When the loop is reached again (e.g. it is nested in
 * another loop), the iterable of the previous execution is not used anymore:
 * the variable is forgotten before evaluating the new iterable, so that an
 * object allocated by it (e.g. {@code range(n)}) is not merged with the
 * previous one, which the variable would keep alive.
 */
public class PyIterableInit extends Assignment {

	/**
	 * Builds the assignment.
	 *
	 * @param cfg        the cfg where the assignment is
	 * @param location   the location of the assignment
	 * @param target     the variable holding the iterable
	 * @param expression the iterable
	 */
	public PyIterableInit(
			CFG cfg,
			CodeLocation location,
			VariableRef target,
			Expression expression) {
		super(cfg, location, target, expression);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		VariableRef target = (VariableRef) getLeft();
		return super.forwardSemantics(entryState.forgetIdentifier(target.getIdentifier(), this), interprocedural,
				expressions);
	}
}
