package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.util.collections.CollectionsDiffBuilder;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;

public class FromImport extends Statement {

	private final String lib;
	private final Map<String, String> components;

	// from <lib> import <left> as <right>
	public FromImport(
			Program program,
			String lib,
			Map<String, String> components,
			CFG cfg,
			CodeLocation loc) {
		super(cfg, loc);
		this.lib = lib;
		this.components = components;
		//LibrarySpecificationProvider.importLibrary(program, lib);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		FromImport other = (FromImport) o;
		int cmp;
		if ((cmp = lib.compareTo(other.lib)) != 0)
			return cmp;
		if ((cmp = Integer.compare(components.keySet().size(), other.components.keySet().size())) != 0)
			return cmp;

		CollectionsDiffBuilder<String> builder = new CollectionsDiffBuilder<>(
				String.class,
				components.keySet(),
				other.components.keySet());
		builder.compute(String::compareTo);

		if (!builder.sameContent())
			// same size means that both have at least one element that is
			// different
			return builder.getOnlyFirst().iterator().next().compareTo(builder.getOnlySecond().iterator().next());

		// same keys: just iterate over them and apply comparisons
		// since fields is sorted, the order of iteration will be consistent
		System.out.println(this.components.entrySet());
		for (Entry<String, String> entry : this.components.entrySet()) {
			if (entry.getValue() != null) {
				if ((cmp = entry.getValue().compareTo(other.components.get(entry.getKey()))) != 0)
					return cmp;
			}
		}
		return 0;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		StringBuilder builder = new StringBuilder("from ").append(lib).append(" import ");
		for (Entry<String, String> component : components.entrySet()) {
			builder.append(component.getKey());
			if (component.getValue() != null)
				builder.append(" as ").append(component.getValue());
			builder.append(", ");
		}
		builder.delete(builder.length() - 2, builder.length());
		return builder.toString();
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(AnalysisState<A> entryState, InterproceduralAnalysis<A, D> interprocedural, StatementStore<A> expressions) throws SemanticException {
		AnalysisState<A> result = interprocedural.getAnalysis().smallStepSemantics(entryState, new Skip(getLocation()), this);

		if (result.getExecutionInfo(SymbolAliasing.INFO_KEY) == null)
			result = result.storeExecutionInfo(SymbolAliasing.INFO_KEY, new SymbolAliasing());

		for (Entry<String, String> component : components.entrySet()) {
			if (component.getValue() != null)
				result = result.storeExecutionInfo(SymbolAliasing.INFO_KEY,
						result.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).alias(
								new QualifiedNameSymbol(lib, component.getKey()),
								new QualifiedNameSymbol(null, component.getValue())));
			else
				result = result.storeExecutionInfo(SymbolAliasing.INFO_KEY,
						result.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).alias(
								new QualifiedNameSymbol(lib, component.getKey()),
								new QualifiedNameSymbol(null, component.getKey())));
		}
		//PythonModuleLattice newModule = new PythonModuleLattice(false, new PythonModule());
		//result = result.storeExecutionInfo("meta_sys_modules", result.getExecutionInfo(MetaSysModules.INFO_KEY, MetaSysModules.class).putModule()
		return result;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + Objects.hash(components, lib);
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
		FromImport other = (FromImport) obj;
		return Objects.equals(components, other.components) && Objects.equals(lib, other.lib);
	}

}
