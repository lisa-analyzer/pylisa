package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
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
import it.unive.lisa.util.collections.CollectionsDiffBuilder;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.namespaces.Namespaces;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.PyNameRef;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;

public class FromImport extends Statement {

	private final String lib;
	private final Map<String, String> components;

	// from <lib> import <left> as <right>
	// the names bound by the statement
	private final List<PyNameRef> bound;

	public FromImport(
			Program program,
			String lib,
			Map<String, String> components,
			CFG cfg,
			CodeLocation loc) {
		this(program, lib, components, List.of(), cfg, loc);
	}

	/**
	 * Builds the statement.
	 *
	 * @param program    the program
	 * @param lib        the imported module
	 * @param components the imported names, mapped to their aliases (or
	 *                       {@code null}), or {@code * -> *} for
	 *                       {@code from lib import *}
	 * @param bound      the names bound by the statement
	 * @param cfg        the cfg where the statement is
	 * @param loc        the location of the statement
	 */
	public FromImport(
			Program program,
			String lib,
			Map<String, String> components,
			List<PyNameRef> bound,
			CFG cfg,
			CodeLocation loc) {
		super(cfg, loc);
		this.lib = lib;
		this.components = components;
		this.bound = bound;
		LibrarySpecificationProvider.importLibrary(program, lib);
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
		for (Entry<String, String> entry : this.components.entrySet())
			// the alias is null for components imported without "as"
			if ((cmp = Objects.compare(entry.getValue(), other.components.get(entry.getKey()),
					Comparator.nullsFirst(Comparator.naturalOrder()))) != 0)
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
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<
				A> result = interprocedural.getAnalysis().smallStepSemantics(entryState, new Skip(getLocation()), this);

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

		return bind(result, bound, components.containsKey("*"), getCFG());
	}

	/**
	 * Binds the names bound by an import statement. A star import
	 * ({@code from m import *}) might bind the given names, if they are known
	 * (see {@code LocalModules}), and any name of the module otherwise.
	 *
	 * @param <A>   the kind of {@link AbstractLattice} of the state
	 * @param state the state
	 * @param bound the names bound by the statement
	 * @param star  whether this is a star import
	 * @param cfg   the cfg where the statement is
	 *
	 * @return the updated state
	 */
	static <A extends AbstractLattice<A>> AnalysisState<A> bind(
			AnalysisState<A> state,
			List<PyNameRef> bound,
			boolean star,
			CFG cfg)
			throws SemanticException {
		AnalysisState<A> result = state;
		for (PyNameRef name : bound)
			result = name.bind(result);
		if (star && !bound.isEmpty())
			// the names of the module are known, but the module might not
			// bind all of them (e.g. if some are bound inside an if)
			return state.lub(result);
		Namespaces namespaces = Namespaces.of(result);
		String module = PyCFG.moduleOf(cfg);
		if (star && namespaces != null && module != null && !result.getExecutionState().isBottom())
			result = Namespaces.store(result, namespaces.havocModule(module));
		return result;
	}
}
