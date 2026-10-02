package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.cfg.type.PyModuleType;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import java.util.List;
import org.apache.commons.lang3.tuple.Pair;

public class FromImport extends Expression {

	private final String sourceModuleName;
	private final List<Pair<String, String>> imports; // (localAlias,
														// originalName)
	private final ModuleUnit currentModule;

	public FromImport(
			CFG cfg,
			CodeLocation loc,
			ModuleUnit currentModule,
			String sourceModuleName,
			List<Pair<String, String>> imports) {
		super(cfg, loc);
		this.currentModule = currentModule;
		this.sourceModuleName = sourceModuleName;
		this.imports = imports;
	}

	@Override
	public String toString() {
		String names = imports.stream()
				.map(Pair::getLeft)
				.reduce((
						a,
						b) -> a + ", " + b)
				.orElse("");
		return "from " + sourceModuleName + " import " + names;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> state = entryState;
		if (PyModuleType.isRegistered(sourceModuleName))
			state = ObjectRegister.initialize(state, this,
					PyModuleType.lookup(sourceModuleName).getUnit(), interprocedural, expressions);

		for (Pair<String, String> entry : imports) {
			String alias = entry.getLeft();
			String originalName = entry.getRight();
			String qualifiedName = sourceModuleName + "." + originalName;
			Expression target = makeTarget(alias);
			// Class resolution is base-name-driven: conditionally redefined
			// classes share a qualified name but live under distinct
			// allocation-site identities (e.g. `X.Y@24:4` vs. `X.Y@38:4`).
			// Collect every matching def-site and lattice-join the per-unit
			// assignments so the target binds a set of ClassLiterals.
			java.util.Collection<
					it.unive.pylisa.cfg.type.PyClassType> classMatches = it.unive.pylisa.cfg.type.PyClassType
							.lookupAllByBaseName(qualifiedName);
			if (!classMatches.isEmpty()) {
				AnalysisState<A> joined = state.bottom();
				for (it.unive.pylisa.cfg.type.PyClassType t : classMatches) {
					var unit = t.getUnit();
					AnalysisState<A> branch = ObjectRegister.initialize(state, this, unit, interprocedural,
							expressions);
					Expression value = new ClassLiteral(getCFG(), getLocation(), unit);
					branch = new PyAssign(getCFG(), getLocation(), target, value)
							.forwardSemantics(branch, interprocedural, expressions);
					joined = joined.lub(branch);
				}
				state = joined;
				continue;
			}
			Expression value;
			if (PyFunctionType.isRegistered(qualifiedName)) {
				FunctionUnit unit = (FunctionUnit) PyFunctionType.lookup(qualifiedName).getUnit();
				state = ObjectRegister.initialize(state, this, unit, interprocedural, expressions);
				value = new ImportFunction(getCFG(), getLocation(), qualifiedName, unit);
			} else if (PyModuleType.isRegistered(qualifiedName)) {
				var unit = PyModuleType.lookup(qualifiedName).getUnit();
				state = ObjectRegister.initialize(state, this, unit, interprocedural, expressions);
				value = new ModuleLiteral(getCFG(), getLocation(), unit);
			} else if (PyModuleType.isRegistered(sourceModuleName)
					&& !PyModuleType.lookup(sourceModuleName).isUnknown()) {
				// Source module is a known project/library module — the member
				// is a
				// module-level variable. Read it directly from the module's
				// scope
				// without overwriting its value with PushAny.
				var sourceModule = PyModuleType.lookup(sourceModuleName).getUnit();
				value = new PythonScopedAttributeAccessRef(getCFG(), getLocation(), sourceModule,
						new it.unive.lisa.program.Global(getLocation(), sourceModule, originalName, false));
			} else {
				value = new UnknownAttributeSymbolRef(getCFG(), getLocation(), sourceModuleName, originalName);
			}
			state = new PyAssign(getCFG(), getLocation(), target, value)
					.forwardSemantics(state, interprocedural, expressions);
		}
		return state;
	}

	private Expression makeTarget(
			String name) {
		if (currentModule != null)
			return new PythonScopedAttributeAccessRef(getCFG(), getLocation(),
					currentModule, new Global(getLocation(), currentModule, name, false));
		return new VariableRef(getCFG(), getLocation(), name);
	}

	@Override
	public boolean equals(
			Object obj) {
		return this == obj;
	}

	@Override
	public int hashCode() {
		return System.identityHashCode(this);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		FromImport other = (FromImport) o;
		int cmp = sourceModuleName.compareTo(other.sourceModuleName);
		if (cmp != 0)
			return cmp;
		cmp = Integer.compare(imports.size(), other.imports.size());
		if (cmp != 0)
			return cmp;
		for (int i = 0; i < imports.size(); i++) {
			Pair<String, String> t = imports.get(i);
			Pair<String, String> u = other.imports.get(i);
			cmp = t.getLeft().compareTo(u.getLeft());
			if (cmp != 0)
				return cmp;
			cmp = t.getRight().compareTo(u.getRight());
			if (cmp != 0)
				return cmp;
		}
		return Integer.compare(System.identityHashCode(this), System.identityHashCode(other));
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}
}
