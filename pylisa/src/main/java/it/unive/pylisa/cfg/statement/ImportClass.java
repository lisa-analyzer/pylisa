package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.expression.PyAssign;
import java.util.Comparator;
import java.util.Objects;

public class ImportClass extends Expression {
	private String className;
	private CompilationUnit classUnit;

	/**
	 * Builds a statement happening at the given source location.
	 *
	 * @param cfg      the cfg that this statement belongs to
	 * @param cfg      the cfg that this statement belongs to
	 * @param location the location where this statement is defined within the
	 *                     program
	 */
	protected ImportClass(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location);
	}

	public ImportClass(
			CFG cfg,
			CodeLocation location,
			String className,
			CompilationUnit classUnit) {
		this(cfg, location);
		this.className = className;
		this.classUnit = classUnit;

	}

	@Override
	public String toString() {
		return "<class> " + className;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> state = entryState;
		state = ObjectRegister.initialize(state, this, classUnit, interprocedural, expressions);
		VariableRef v = new VariableRef(this.getCFG(), getLocation(), "$" + classUnit.getName());
		PyAssign assign = new PyAssign(this.getCFG(), getLocation(), v,
				new ClassLiteral(this.getCFG(), getLocation(), classUnit));
		state = assign.forwardSemantics(state, interprocedural, expressions);
		return state;
	}

	/**
	 * Orders the bindings of classes by the qualified name of the class they bind: pylisa builds
	 * them with one synthetic location, and the classes of one module must stay distinct statements.
	 */
	@Override
	protected int compareSameClass(
			Statement o) {
		return Objects.compare(boundName(), ((ImportClass) o).boundName(),
				Comparator.nullsFirst(Comparator.naturalOrder()));
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj
				|| super.equals(obj) && getClass() == obj.getClass()
						&& Objects.equals(boundName(), ((ImportClass) obj).boundName());
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + Objects.hashCode(boundName());
	}

	private String boundName() {
		return classUnit == null ? null : classUnit.getName();
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}
}
