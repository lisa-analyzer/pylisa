package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.expression.PyAssign;

public class ImportModule extends Expression {
	private String moduleName;
	private CompilationUnit pythonModuleUnit;

	/**
	 * Builds a statement happening at the given source location.
	 *
	 * @param cfg      the cfg that this statement belongs to
	 * @param cfg      the cfg that this statement belongs to
	 * @param location the location where this statement is defined within the
	 *                     program
	 */
	protected ImportModule(
			CFG cfg,
			CodeLocation location) {
		super(cfg, location);
	}

	public ImportModule(
			CFG cfg,
			CodeLocation location,
			String moduleName,
			CompilationUnit pythonModuleUnit) {
		this(cfg, location);
		this.moduleName = moduleName;
		this.pythonModuleUnit = pythonModuleUnit;

	}

	@Override
	public String toString() {
		return "<module> " + moduleName;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> state = entryState;
		state = ObjectRegister.initialize(state, this, pythonModuleUnit, interprocedural);
		VariableRef v = new VariableRef(this.getCFG(), getLocation(), "$" + pythonModuleUnit.getName());
		PyAssign assign = new PyAssign(this.getCFG(), getLocation(), v,
				new ModuleLiteral(this.getCFG(), getLocation(), pythonModuleUnit));
		state = assign.forwardSemantics(state, interprocedural, expressions);
		return state;
		// return state;
		// if (state.getExecutionInfo(MetaSysModules.INFO_KEY) == null)
		// state = state.storeExecutionInfo(MetaSysModules.INFO_KEY, new
		// MetaSysModules());
		// state = state.storeExecutionInfo(MetaSysModules.INFO_KEY,
		// state.getExecutionInfo(MetaSysModules.INFO_KEY, MetaSysModules.class)
		// .putModule(moduleName, new PythonModuleLattice(false,
		// pythonModuleUnit)));

		// Type type = PyModuleType.lookup(pythonModuleUnit.getName());

		// PyNewObj pyNewObj = new PyNewObj(this.getCFG(), getLocation(),
		// "$moduleInit", type);
		// return pyNewObj.forwardSemantics(state, interprocedural,
		// expressions);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		return 0;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (!(obj instanceof NaryExpression))
			return false;
		ImportModule other = (ImportModule) obj;
		if (moduleName == null) {
			if (other.moduleName != null)
				return false;
		} else if (!moduleName.equals(other.moduleName))
			return false;
		if (pythonModuleUnit == null) {
			if (other.pythonModuleUnit != null)
				return false;
		} else if (!pythonModuleUnit.equals(other.pythonModuleUnit))
			return false;
		return true;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}
}
