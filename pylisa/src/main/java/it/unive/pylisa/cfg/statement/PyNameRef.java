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
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import java.util.Objects;
import java.util.Set;

/**
 * A Python name reference that performs <em>G → B</em> (global → built-in)
 * scope resolution at <strong>analysis time</strong>.
 * <p>
 * The L (local) step is already handled at parse time by {@code makeRef} via
 * {@code VariableRef}. When the module-scope variable {@code $<module>::<name>}
 * carries no resolved types in the current abstract state, this node
 * transparently evaluates to {@code $builtins::<name>}, implementing the
 * missing B-fallback of Python LEGB scope semantics.
 * <p>
 * Using a dedicated expression node — rather than ad-hoc checks inside
 * {@code FunctionApply} or {@code AttributeAccess} — keeps LEGB logic in a
 * single, well-defined place and leaves room for the E (enclosing) step.
 */
public class PyNameRef extends Expression {

	/** The module name for the G (global) scope, e.g. {@code "__main__"}. */
	private final String moduleName;

	/** The unqualified identifier name, e.g. {@code "staticmethod"}. */
	private final String name;

	/**
	 * Builds a name reference with G → B fallback semantics.
	 *
	 * @param cfg        the CFG this expression belongs to
	 * @param location   source location
	 * @param name       unqualified identifier name
	 * @param moduleName name of the enclosing module (G scope)
	 */
	public PyNameRef(
			CFG cfg,
			CodeLocation location,
			String name,
			String moduleName) {
		super(cfg, location, Untyped.INSTANCE);
		this.name = Objects.requireNonNull(name, "name");
		this.moduleName = Objects.requireNonNull(moduleName, "moduleName");
	}

	/** @return the unqualified identifier name */
	public String getName() {
		return name;
	}

	/** @return the module name used for the G-scope lookup */
	public String getModuleName() {
		return moduleName;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + Objects.hash(moduleName, name);
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
		PyNameRef other = (PyNameRef) obj;
		return Objects.equals(name, other.name) && Objects.equals(moduleName, other.moduleName);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PyNameRef other = (PyNameRef) o;
		int cmp = moduleName.compareTo(other.moduleName);
		if (cmp != 0)
			return cmp;
		return name.compareTo(other.name);
	}

	@Override
	public String toString() {
		return moduleName + "::" + name;
	}

	/**
	 * Evaluates this name reference with G → B fallback:
	 * <ol>
	 * <li>Looks up {@code $<moduleName>::<name>} in the abstract state
	 * (G).</li>
	 * <li>If the result is unresolved (empty or all
	 * {@link it.unive.pylisa.program.type.NoInfoType}), falls back to
	 * {@code $builtins::<name>} (B).</li>
	 * </ol>
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> state,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();

		// G: module-scope lookup
		GlobalVariable moduleVar = new GlobalVariable(
				Untyped.INSTANCE, "$" + moduleName + "::" + name, getLocation());
		Set<Type> gTypes = analysis.getRuntimeTypesOf(state, moduleVar, this);

		if (!UnknownSymbolUtils.isUnresolvedTypeSet(gTypes))
			return analysis.smallStepSemantics(state, moduleVar, this);

		// B: builtins fallback
		GlobalVariable builtinsVar = new GlobalVariable(
				Untyped.INSTANCE, "$builtins::" + name, getLocation());
		return analysis.smallStepSemantics(state, builtinsVar, this);
	}
}
