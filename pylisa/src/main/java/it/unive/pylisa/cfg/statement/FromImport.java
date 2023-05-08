package it.unive.pylisa.cfg.statement;

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
import it.unive.lisa.symbolic.value.Skip;

public class FromImport extends Import {
	private final String component;

	public FromImport(String importedLibrary, String component, String asName, CFG cfg, CodeLocation loc) {
		super(importedLibrary, asName, cfg, loc);
		this.component = component;
	}

	@Override
	public String toString() {
		if (name == null)
			return "from " + super.importedLibrary + " import " + component;
		return "from " + super.importedLibrary + " import " + component + " as " + name;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((component == null) ? 0 : component.hashCode());
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
		if (component == null) {
			if (other.component != null)
				return false;
		} else if (!component.equals(other.component))
			return false;
		return true;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> semantics(AnalysisState<A, H, V, T> entryState,
					InterproceduralAnalysis<A, H, V, T> interprocedural, StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		entryState = entryState.smallStepSemantics(new Skip(getLocation()), this);

		if (name == null)
			return entryState.alias(new QualifiedNameSymbol(importedLibrary, component),
					new QualifiedNameSymbol(null, name));

		return entryState.alias(new QualifiedNameSymbol(importedLibrary, component),
				new QualifiedNameSymbol(null, component));
	}
}
