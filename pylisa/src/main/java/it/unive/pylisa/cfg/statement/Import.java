package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.symbols.QualifierSymbol;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
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

public class Import extends Statement {

	private final Map<String, String> libs;

	// import <left> as <right>
	public Import(Program program, Map<String, String> libs, CFG cfg, CodeLocation loc) {
		super(cfg, loc);
		this.libs = libs;
		for (String lib : libs.keySet())
			LibrarySpecificationProvider.importLibrary(program, lib);
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
		StringBuilder builder = new StringBuilder("import ");
		for (Entry<String, String> lib : libs.entrySet()) {
			builder.append(lib.getKey());
			if (lib.getValue() != null)
				builder.append(" as ").append(lib.getValue());
			builder.append(", ");
		}
		builder.delete(builder.length() - 2, builder.length());
		return builder.toString();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((libs == null) ? 0 : libs.hashCode());
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
		Import other = (Import) obj;
		if (libs == null) {
			if (other.libs != null)
				return false;
		} else if (!libs.equals(other.libs))
			return false;
		return true;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> semantics(
					AnalysisState<A, H, V, T> entryState,
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		AnalysisState<A, H, V, T> result = entryState.smallStepSemantics(new Skip(getLocation()), this);

		for (Entry<String, String> lib : libs.entrySet()) {
			if (lib.getValue() != null)
				result = result.alias(new QualifierSymbol(lib.getKey()), new QualifierSymbol(lib.getValue()));
		}

		return result;
	}
}
