package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.FixpointInfo;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.namespaces.Namespaces;

/**
 * A synthetic statement at the beginning of the code of each module, that
 * ensures that the {@link Namespaces} are part of the {@link FixpointInfo} of
 * the analysis state, storing {@link Namespaces#EMPTY} if they are missing
 * (e.g. when the module is an entry point of the analysis). The
 * {@link FixpointInfo} treats a missing key as carrying no information, so that
 * joining a state without the namespaces with a state having them would yield
 * the latter, losing the names that are not bound in the former: this statement
 * guarantees that all the states of the module (and of the functions it calls)
 * contain the namespaces.
 */
public class PyNamespacesInit extends Statement {

	/**
	 * Builds the statement.
	 *
	 * @param cfg      the cfg of the code of the module
	 * @param location the location of the statement
	 */
	public PyNamespacesInit(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		return 0; // no extra fields to compare
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		return "init-namespaces";
	}

	@Override
	public int hashCode() {
		return super.hashCode() ^ getClass().getName().hashCode();
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		return getClass() == obj.getClass();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = interprocedural.getAnalysis().smallStepSemantics(entryState,
				new Skip(getLocation()), this);
		if (Namespaces.of(result) == null)
			result = Namespaces.store(result, Namespaces.EMPTY);
		return result;
	}
}
