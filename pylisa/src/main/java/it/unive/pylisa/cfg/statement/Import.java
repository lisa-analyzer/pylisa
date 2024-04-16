package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.symbols.QualifierSymbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.CFGCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.util.collections.CollectionsDiffBuilder;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

public class Import extends Statement {

	private final Map<String, String> libs;
	private List<UnresolvedCall> cfgCalls;
	// import <left> as <right>
	public Import(
			Program program,
			Map<String, String> libs,
			CFG cfg,
			CodeLocation loc) {
		super(cfg, loc);
		this.libs = libs;
		for (String lib : libs.keySet())
			LibrarySpecificationProvider.importLibrary(program, lib);
	}

	public Import(
			Program program,
			Map<String, String> libs,
			CFG cfg,
			CodeLocation loc,
			List<UnresolvedCall> cfgCalls) {
		this(program, libs, cfg, loc);
		this.cfgCalls = cfgCalls;
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		Import other = (Import) o;
		int cmp;
		if ((cmp = Integer.compare(libs.keySet().size(), other.libs.keySet().size())) != 0)
			return cmp;

		CollectionsDiffBuilder<String> builder = new CollectionsDiffBuilder<>(
				String.class,
				libs.keySet(),
				other.libs.keySet());
		builder.compute(String::compareTo);

		if (!builder.sameContent())
			// same size means that both have at least one element that is
			// different
			return builder.getOnlyFirst().iterator().next().compareTo(builder.getOnlySecond().iterator().next());

		// same keys: just iterate over them and apply comparisons
		// since fields is sorted, the order of iteration will be consistent
		for (Entry<String, String> entry : this.libs.entrySet())
			if ((cmp = entry.getValue().compareTo(other.libs.get(entry.getKey()))) != 0)
				return cmp;

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
	public boolean equals(
			Object obj) {
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
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> result = entryState.smallStepSemantics(new Skip(getLocation()), this);

		if (result.getInfo(SymbolAliasing.INFO_KEY) == null)
			result = result.storeInfo(SymbolAliasing.INFO_KEY, new SymbolAliasing());

		for (Entry<String, String> lib : libs.entrySet()) {
			if (lib.getValue() != null)
				result = result.storeInfo(SymbolAliasing.INFO_KEY,
						result.getInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class)
								.alias(new QualifierSymbol(lib.getKey()), new QualifierSymbol(lib.getValue())));
		}
		for (UnresolvedCall uc : cfgCalls) {
			uc.forwardSemanticsAux(interprocedural, result, new ExpressionSet[]{}, expressions);
		}
		return result;
	}
}
