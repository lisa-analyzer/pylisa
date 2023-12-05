package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
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
		LibrarySpecificationProvider.importLibrary(program, lib);
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

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = entryState.smallStepSemantics(new Skip(getLocation()), this);

		if (result.getInfo(SymbolAliasing.INFO_KEY) == null)
			result = result.storeInfo(SymbolAliasing.INFO_KEY, new SymbolAliasing());

		for (Entry<String, String> component : components.entrySet()) {
			if (component.getValue() != null)
				result = result.storeInfo(SymbolAliasing.INFO_KEY,
						result.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).alias(
								new QualifiedNameSymbol(lib, component.getKey()),
								new QualifiedNameSymbol(null, component.getValue())));
			else
				result = result.storeInfo(SymbolAliasing.INFO_KEY,
						result.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).alias(
								new QualifiedNameSymbol(lib, component.getKey()),
								new QualifiedNameSymbol(null, component.getKey())));
		}

		return result;
	}
}
