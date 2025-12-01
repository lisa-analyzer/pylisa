package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.PyCFG;

public class FunctionDef extends Statement {
	PyCFG method;

	public FunctionDef(
			CFG cfg,
			CodeLocation location,
			PyCFG method) {
		super(cfg, location);
		this.method = method;
	}

	@Override
	public String toString() {
		return "def " + method.toString();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(AnalysisState<A> entryState, InterproceduralAnalysis<A, D> interprocedural, StatementStore<A> expressions) throws SemanticException {
		// keep the state as-is
		return entryState;
	}

	@Override
	public CodeLocation getLocation() {
		return null;
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		FunctionDef other = (FunctionDef) o;
		if (method.equals(other.method)) {
			return 0;
		}
		return 1;
	}

	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		FunctionDef other = (FunctionDef) obj;
		return method.equals(other.method);
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

}
