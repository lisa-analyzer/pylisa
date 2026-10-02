package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.program.FunctionUnit;
import java.util.Comparator;
import java.util.Objects;

public class ImportFunction extends Expression {
	private String className;
	private FunctionUnit functionUnit;

	/**
	 * Builds a statement happening at the given source location.
	 *
	 * @param cfg      the cfg that this statement belongs to
	 * @param cfg      the cfg that this statement belongs to
	 * @param location the location where this statement is defined within the
	 *                     program
	 */
	protected ImportFunction(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location);
	}

	public ImportFunction(
			CFG cfg,
			CodeLocation location,
			String className,
			FunctionUnit functionUnit) {
		this(cfg, location);
		this.className = className;
		this.functionUnit = functionUnit;

	}

	@Override
	public String toString() {
		return "<function> " + className;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> state = entryState;
		state = ObjectRegister.initialize(state, this, functionUnit, interprocedural, expressions);
		VariableRef v = new VariableRef(this.getCFG(), getLocation(), "$" + functionUnit.getName());
		PyAssign assign = new PyAssign(this.getCFG(), getLocation(), v,
				new FunctionLiteral(this.getCFG(), getLocation(), functionUnit));
		state = assign.forwardSemantics(state, interprocedural, expressions);
		return state;
	}

	/**
	 * Orders the bindings of functions by the qualified name of the function they bind: pylisa
	 * builds them with one synthetic location, and the methods of one class body must stay distinct
	 * statements. The name given to the statement is not a key: the library loader gives every
	 * method of a class the name of the class.
	 */
	@Override
	protected int compareSameClass(
			Statement o) {
		return Objects.compare(boundName(), ((ImportFunction) o).boundName(),
				Comparator.nullsFirst(Comparator.naturalOrder()));
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj
				|| super.equals(obj) && getClass() == obj.getClass()
						&& Objects.equals(boundName(), ((ImportFunction) obj).boundName());
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + Objects.hashCode(boundName());
	}

	private String boundName() {
		return functionUnit == null ? null : functionUnit.getName();
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	public FunctionUnit getFunctionUnit() {
		return functionUnit;
	}
}
