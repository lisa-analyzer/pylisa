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
 * contain the namespaces. It also records that the code of the module has
 * started (see {@link Namespaces#withStarted(String)}).
 */
public class PyNamespacesInit extends Statement {

	// the (dotted) name of the module
	private final String module;

	// whether the globals of the module are never bound by its functions
	private final boolean stable;

	/**
	 * Builds the statement.
	 *
	 * @param cfg      the cfg of the code of the module
	 * @param location the location of the statement
	 * @param module   the (dotted) name of the module
	 * @param stable   whether the globals of the module are never bound by its
	 *                     functions (see {@link Namespaces#withStable(String)})
	 */
	public PyNamespacesInit(
			CFG cfg,
			CodeLocation location,
			String module,
			boolean stable) {
		super(cfg, location);
		this.module = module;
		this.stable = stable;
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PyNamespacesInit other = (PyNamespacesInit) o;
		int cmp = module.compareTo(other.module);
		return cmp != 0 ? cmp : Boolean.compare(stable, other.stable);
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
		return super.hashCode() ^ getClass().getName().hashCode() ^ module.hashCode() ^ Boolean.hashCode(stable);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		PyNamespacesInit other = (PyNamespacesInit) obj;
		return module.equals(other.module) && stable == other.stable;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = interprocedural.getAnalysis().smallStepSemantics(entryState,
				new Skip(getLocation()), this);
		Namespaces namespaces = Namespaces.of(result);
		if (namespaces == null)
			namespaces = Namespaces.EMPTY;
		if (stable)
			namespaces = namespaces.withStable(module);
		return Namespaces.store(result, namespaces.withStarted(module));
	}
}
