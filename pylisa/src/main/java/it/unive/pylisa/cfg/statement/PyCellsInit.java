package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.expression.Cells;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.List;

/**
 * A synthetic statement at the beginning of a function whose local variables
 * are captured by nested functions (see {@link Cells}): it allocates the object
 * holding the cells of the function, referenced by {@link Cells#CELLS}, and it
 * copies the values of the parameters that are cells into it.
 */
public class PyCellsInit extends Statement {

	private final PyClassType type;

	private final List<String> parameters;

	/**
	 * Builds the statement.
	 *
	 * @param cfg        the cfg of the function
	 * @param location   the location of the statement
	 * @param type       the type of the object holding the cells
	 * @param parameters the parameters of the function that are cells
	 */
	public PyCellsInit(
			CFG cfg,
			CodeLocation location,
			PyClassType type,
			List<String> parameters) {
		super(cfg, location);
		this.type = type;
		this.parameters = parameters;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		AnalysisState<A> state = Cells.allocate(analysis, entryState, type, Cells.CELLS, loc, this);
		for (String parameter : parameters)
			state = analysis.assign(state, Cells.cell(Cells.CELLS, parameter, loc),
					new Variable(Untyped.INSTANCE, parameter, loc), this);
		return analysis.smallStepSemantics(state, new Skip(loc), this);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PyCellsInit other = (PyCellsInit) o;
		int cmp = type.toString().compareTo(other.type.toString());
		return cmp != 0 ? cmp : parameters.toString().compareTo(other.parameters.toString());
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		return "init-cells" + parameters;
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + type.hashCode() + parameters.hashCode();
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
		PyCellsInit other = (PyCellsInit) obj;
		return type.equals(other.type) && parameters.equals(other.parameters);
	}
}
