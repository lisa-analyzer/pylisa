package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;

public class PythonScopedAttributeAccessRef extends Expression {

	/**
	 * The receiver of the access
	 */
	private final CompilationUnit containerUnit;

	/**
	 * The global being accessed
	 */
	private final Global target;

	/**
	 * Builds the global access, happening at the given location in the program.
	 * The type of this expression is the one of the accessed global.
	 *
	 * @param cfg           the cfg that this expression belongs to
	 * @param location      the location where the expression is defined within
	 *                          the program
	 * @param containerUnit the unit containing the accessed global
	 * @param target        the accessed global
	 */
	public PythonScopedAttributeAccessRef(
			CFG cfg,
			CodeLocation location,
			CompilationUnit containerUnit,
			Global target) {
		super(cfg, location, target.getStaticType());
		this.containerUnit = containerUnit;
		this.target = target;
	}

	/**
	 * Yields the {@link Unit} where the global targeted by this access is
	 * defined.
	 *
	 * @return the container of the global
	 */
	public CompilationUnit getContainerUnit() {
		return containerUnit;
	}

	/**
	 * Yields the {@link Global} targeted by this expression.
	 *
	 * @return the global
	 */
	public Global getTarget() {
		return target;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((containerUnit == null) ? 0 : containerUnit.hashCode());
		result = prime * result + ((target == null) ? 0 : target.hashCode());
		return result;
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
		PythonScopedAttributeAccessRef other = (PythonScopedAttributeAccessRef) obj;
		if (containerUnit == null) {
			if (other.containerUnit != null)
				return false;
		} else if (!containerUnit.equals(other.containerUnit))
			return false;
		if (target == null) {
			if (other.target != null)
				return false;
		} else if (!target.equals(other.target))
			return false;
		return true;
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PythonScopedAttributeAccessRef other = (PythonScopedAttributeAccessRef) o;
		int cmp;
		if ((cmp = containerUnit.getName().compareTo(other.containerUnit.getName())) != 0)
			return cmp;
		return target.getName().compareTo(other.target.getName());
	}

	@Override
	public String toString() {
		return containerUnit.getName() + "::" + target.getName();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> state,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();

		GlobalVariable access = new GlobalVariable(
				target.getStaticType(),
				"$" + containerUnit.getName() + "::" + target.getName(),
				target.getAnnotations(),
				getLocation());
		return analysis.smallStepSemantics(state, access, this);
	}
}
