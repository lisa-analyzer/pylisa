package it.unive.pylisa.libraries.natives;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import java.util.Collections;
import java.util.Iterator;
import java.util.Objects;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * Stands for a modelled call on a branch that depends on assumptions of the
 * environment the analysed program runs in: the errors the model raises there
 * have it as thrower, so that they stay apart from the errors the call raises
 * whatever the assumptions. It is never evaluated and belongs to no CFG's
 * nodes; its parent is the call, which it never changes, so an error it
 * throws is the call's for everything that follows the chain of parents.
 */
public final class AssumptionBranch extends Expression {

	private final Statement call;

	private final SortedSet<String> assumptions;

	/**
	 * Builds the statement.
	 *
	 * @param call        the modelled call
	 * @param assumptions the names of the assumptions the branch depends on,
	 *                        at least one
	 */
	public AssumptionBranch(
			Statement call,
			Set<String> assumptions) {
		super(call.getCFG(), call.getLocation());
		if (assumptions.isEmpty())
			throw new IllegalArgumentException("A branch of " + call + " depends on no assumption");
		this.call = call;
		this.assumptions = Collections.unmodifiableSortedSet(new TreeSet<>(assumptions));
		setParentStatement(call);
	}

	/**
	 * Yields the names of the assumptions the branch depends on.
	 *
	 * @return the names, sorted
	 */
	public SortedSet<String> assumptions() {
		return assumptions;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions) {
		throw new IllegalStateException("The branch of " + call + " under " + assumptions + " is not evaluated");
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		AssumptionBranch other = (AssumptionBranch) o;
		int byCall = call.compareTo(other.call);
		if (byCall != 0)
			return byCall;
		// name by name, so that the order agrees with equals whatever the names
		Iterator<String> mine = assumptions.iterator();
		Iterator<String> theirs = other.assumptions.iterator();
		while (mine.hasNext() && theirs.hasNext()) {
			int byName = mine.next().compareTo(theirs.next());
			if (byName != 0)
				return byName;
		}
		return Boolean.compare(mine.hasNext(), theirs.hasNext());
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj || obj instanceof AssumptionBranch other && call.equals(other.call)
				&& assumptions.equals(other.assumptions);
	}

	@Override
	public int hashCode() {
		return Objects.hash(call, assumptions);
	}

	@Override
	public String toString() {
		return call + " assuming " + assumptions + " does not hold";
	}
}
