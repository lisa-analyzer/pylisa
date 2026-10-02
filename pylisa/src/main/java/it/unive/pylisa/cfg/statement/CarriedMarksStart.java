package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.libraries.natives.CarriedMarks;
import it.unive.pylisa.libraries.natives.Expressions;
import it.unive.pylisa.program.PyProgram;

/**
 * Sets the flag of every carried mark of the program (see
 * {@link CarriedMarks}) to {@code false}, at the program's entry, so that
 * every execution has each flag and a join never meets a flag set on one side
 * only.
 */
public final class CarriedMarksStart extends Expression {

	/**
	 * Builds the statement.
	 *
	 * @param cfg      the CFG of the program's entry
	 * @param location its location
	 */
	public CarriedMarksStart(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> state = entryState;
		Expressions build = new Expressions(getLocation());
		for (String name : PyProgram.setting(this, CarriedMarks.class).orElse(CarriedMarks.NONE).names())
			state = interprocedural.getAnalysis().assign(state, CarriedMarks.flag(name, getLocation()),
					build.bool(false), this);
		return state;
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
		return 0;
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj || obj instanceof CarriedMarksStart other && getLocation().equals(other.getLocation());
	}

	@Override
	public int hashCode() {
		return getLocation().hashCode();
	}

	@Override
	public String toString() {
		return "carried marks unset";
	}
}
