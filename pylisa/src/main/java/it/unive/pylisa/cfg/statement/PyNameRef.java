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
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyModuleType;
import java.util.Objects;
import java.util.Set;

/**
 * A Python name reference that performs <em>G → B</em> (global → built-in)
 * scope resolution at <strong>analysis time</strong>, with an optional
 * qualified-name hint fallback for names known at parse time to be imported
 * from a specific module.
 * <p>
 * The L (local) step is already handled at parse time by {@code makeRef} via
 * {@code VariableRef}. When the module-scope variable {@code $<module>::<name>}
 * carries no resolved types in the current abstract state, this node
 * transparently evaluates to {@code $builtins::<name>}. If that still yields no
 * type information and a {@code qualifiedImportHint} (e.g.
 * {@code "fastapi.APIRouter"}) is available, the hint is resolved against the
 * {@code PyClassType}/{@code PyFunctionType}/{@code PyModuleType} registries —
 * this recovers the binding in cases where {@code from X import Y} ran but its
 * {@code PyAssign} side-effect did not propagate through the TypeEnvironment
 * (observed in deeply-nested submodule inits, e.g. dispatch).
 */
public class PyNameRef extends Expression {

	/** The module name for the G (global) scope, e.g. {@code "__main__"}. */
	private final String moduleName;

	/** The unqualified identifier name, e.g. {@code "staticmethod"}. */
	private final String name;

	/**
	 * Optional hint captured at parse time: the fully-qualified name this
	 * identifier resolves to via a {@code from X import Y} statement in the
	 * current module. Used as a last-resort fallback when both G and B scope
	 * lookups fail to produce a typed value.
	 */
	private final String qualifiedImportHint;

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
		this(cfg, location, name, moduleName, null);
	}

	/**
	 * Builds a name reference with G → B fallback semantics and an optional
	 * qualified-name hint from a {@code from X import Y} statement.
	 *
	 * @param cfg                 the CFG this expression belongs to
	 * @param location            source location
	 * @param name                unqualified identifier name
	 * @param moduleName          name of the enclosing module (G scope)
	 * @param qualifiedImportHint fully-qualified name (e.g.
	 *                                {@code "fastapi.APIRouter"}), or
	 *                                {@code null}
	 */
	public PyNameRef(
			CFG cfg,
			CodeLocation location,
			String name,
			String moduleName,
			String qualifiedImportHint) {
		super(cfg, location, Untyped.INSTANCE);
		this.name = Objects.requireNonNull(name, "name");
		this.moduleName = Objects.requireNonNull(moduleName, "moduleName");
		this.qualifiedImportHint = qualifiedImportHint;
	}

	/** @return the unqualified identifier name */
	public String getName() {
		return name;
	}

	/** @return the module name used for the G-scope lookup */
	public String getModuleName() {
		return moduleName;
	}

	/**
	 * @return the fully-qualified import hint, or {@code null} if unknown
	 */
	public String getQualifiedImportHint() {
		return qualifiedImportHint;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + Objects.hash(moduleName, name, qualifiedImportHint);
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
		return Objects.equals(name, other.name) && Objects.equals(moduleName, other.moduleName)
				&& Objects.equals(qualifiedImportHint, other.qualifiedImportHint);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PyNameRef other = (PyNameRef) o;
		int cmp = moduleName.compareTo(other.moduleName);
		if (cmp != 0)
			return cmp;
		cmp = name.compareTo(other.name);
		if (cmp != 0)
			return cmp;
		String a = qualifiedImportHint == null ? "" : qualifiedImportHint;
		String b = other.qualifiedImportHint == null ? "" : other.qualifiedImportHint;
		return a.compareTo(b);
	}

	@Override
	public String toString() {
		return moduleName + "::" + name;
	}

	/**
	 * Evaluates this name reference with G → B → hint fallback:
	 * <ol>
	 * <li>Looks up {@code $<moduleName>::<name>} in the abstract state
	 * (G).</li>
	 * <li>If unresolved, falls back to {@code $builtins::<name>} (B).</li>
	 * <li>If still unresolved and a {@code qualifiedImportHint} is set,
	 * resolves the hint against the PyClassType / PyFunctionType / PyModuleType
	 * registries and evaluates the corresponding {@code $<qualified>} global
	 * instead.</li>
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
		Set<Type> bTypes = analysis.getRuntimeTypesOf(state, builtinsVar, this);
		if (!UnknownSymbolUtils.isUnresolvedTypeSet(bTypes))
			return analysis.smallStepSemantics(state, builtinsVar, this);

		// Import-hint fallback: recovers bindings from `from X import Y` in
		// submodules where PyAssign's type side-effect didn't propagate. By
		// re-evaluating a fresh ClassLiteral / producing a typed GlobalVariable
		// for the qualified name, the downstream FunctionApply sees a
		// PyClassType / PyFunctionType and dispatches correctly.
		if (qualifiedImportHint != null) {
			// Base-name lookup: a qualified import hint like "X.Y" may map to
			// multiple user-defined class def-sites (conditional redefinition).
			// Lattice-join a ClassLiteral per match so the name evaluates to
			// the union of possible classes.
			java.util.Collection<
					PyClassType> classMatches = PyClassType.lookupAllByBaseName(qualifiedImportHint);
			if (!classMatches.isEmpty()) {
				AnalysisState<A> joined = state.bottom();
				for (PyClassType t : classMatches) {
					var unit = t.getUnit();
					AnalysisState<A> prepared = it.unive.pylisa.analysis.ObjectRegister.initialize(
							state, this, unit, interprocedural, expressions);
					AnalysisState<A> branch = new ClassLiteral(getCFG(), getLocation(), unit)
							.forwardSemantics(prepared, interprocedural, expressions);
					joined = joined.lub(branch);
				}
				return joined;
			}
			if (PyFunctionType.isRegistered(qualifiedImportHint)) {
				// A GlobalVariable whose static type is the registered
				// PyFunctionType lets FunctionApply dispatch to the pluggable.
				return analysis.smallStepSemantics(state,
						new GlobalVariable(PyFunctionType.lookup(qualifiedImportHint),
								"$" + qualifiedImportHint, getLocation()),
						this);
			}
			if (PyModuleType.isRegistered(qualifiedImportHint)) {
				return analysis.smallStepSemantics(state,
						new GlobalVariable(PyModuleType.lookup(qualifiedImportHint),
								"$" + qualifiedImportHint, getLocation()),
						this);
			}
		}

		// Final fallback: evaluate the G-scope variable. When neither G nor B
		// carry runtime type info and no import hint applies, the name is
		// either a module-local whose heap allocation was written by a
		// library-pluggable semantics (e.g. `api_router = APIRouter(...)` —
		// the network domain writes an HttpActiveNode allocation but does
		// not populate the type lattice) or a truly unresolved identifier.
		// In both cases, returning the G-scope identifier preserves the
		// heap link `$<module>::<name> → AllocationSite` that downstream
		// operators (ActiveNodeCompose, ChannelEndpointCreation, instance
		// attribute access) need, and degrades gracefully to an untyped
		// read for genuinely unbound names. Routing unresolved module
		// locals through `$builtins::<name>` would silently break heap
		// dereferences at the next use site.
		return analysis.smallStepSemantics(state, moduleVar, this);
	}
}
