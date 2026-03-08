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
import it.unive.pylisa.cfg.type.PyClassType;
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
			Expression value;
			if (PyClassType.isRegistered(qualifiedName)) {
				var unit = PyClassType.lookup(qualifiedName).getUnit();
				state = ObjectRegister.initialize(state, this, unit, interprocedural, expressions);
				value = new ClassLiteral(getCFG(), getLocation(), unit);
			} else if (PyFunctionType.isRegistered(qualifiedName)) {
				FunctionUnit unit = (FunctionUnit) PyFunctionType.lookup(qualifiedName).getUnit();
				state = ObjectRegister.initialize(state, this, unit, interprocedural, expressions);
				value = new ImportFunction(getCFG(), getLocation(), qualifiedName, unit);
			} else if (PyModuleType.isRegistered(qualifiedName)) {
				var unit = PyModuleType.lookup(qualifiedName).getUnit();
				state = ObjectRegister.initialize(state, this, unit, interprocedural, expressions);
				value = new ModuleLiteral(getCFG(), getLocation(), unit);
			} else {
				value = new UnknownAttributeSymbolRef(getCFG(), getLocation(), sourceModuleName, originalName);
			}
			Expression target = makeTarget(alias);
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
