package it.unive.pylisa.cfg.statement;

import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;

public class FromImport extends Statement {

	private final String lib;
	private final Map<String, String> components;

	// from component import <left> as <right>
	public FromImport(String lib, Map<String, String> components, CFG cfg, CodeLocation loc) {
		super(cfg, loc);
		this.lib = lib;
		this.components = components;
	}

	@Override
	public int setOffset(int i) {
		super.offset = i;
		return i;
	}

	@Override
	public <V> boolean accept(GraphVisitor<CFG, Statement, Edge, V> visitor, V tool) {
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
	public boolean equals(Object obj) {
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
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> semantics(AnalysisState<A, H, V, T> entryState,
					InterproceduralAnalysis<A, H, V, T> interprocedural, StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		AnalysisState<A, H, V, T> result = entryState.smallStepSemantics(new Skip(getLocation()), this);

		for (Entry<String, String> component : components.entrySet()) {
			if (component.getValue() != null)
				result = result.alias(
						new QualifiedNameSymbol(lib, component.getKey()),
						new QualifiedNameSymbol(null, component.getValue()));
			else
				result = result.alias(
						new QualifiedNameSymbol(lib, component.getKey()),
						new QualifiedNameSymbol(null, component.getKey()));
		}

		return result;
	}
}
