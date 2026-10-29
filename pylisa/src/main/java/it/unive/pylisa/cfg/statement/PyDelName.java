package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.namespaces.Binding;
import it.unive.pylisa.cfg.expression.PyNameRef;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * The deletion of names, {@code del x, y}. Names are deleted from left to
 * right: deleting a name that is not bound raises {@code NameError} (for global
 * names) or {@code UnboundLocalError} (for local ones), otherwise the name is
 * unbound, and its value is forgotten.
 */
public class PyDelName extends Statement {

	private final List<PyNameRef> names;

	/**
	 * Builds the statement.
	 *
	 * @param cfg      the cfg where the statement is
	 * @param location the location of the statement
	 * @param names    the names to delete, in order
	 */
	public PyDelName(
			CFG cfg,
			CodeLocation location,
			PyNameRef... names) {
		super(cfg, location);
		this.names = Arrays.asList(names);
	}

	/**
	 * Yields the names deleted by this statement, in order.
	 *
	 * @return the names
	 */
	public List<PyNameRef> getNames() {
		return names;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		AnalysisState<A> state = entryState;
		for (PyNameRef name : names) {
			Binding status = name.status(state);
			PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
			AnalysisState<A> result = state.bottomExecution();
			if (status == null || status == Binding.BOTTOM || status.mightBeBound())
				result = result.lub(name.unbind(state.forgetIdentifier(name.getIdentifier(), this)));
			if (status != null && status.mightBeUnbound())
				name.unboundError(raising, state);
			state = raising.moveFrom(result);
		}
		return analysis.smallStepSemantics(state, new Skip(getLocation()), context, this);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PyDelName other = (PyDelName) o;
		int cmp = Integer.compare(names.size(), other.names.size());
		for (int i = 0; cmp == 0 && i < names.size(); i++)
			cmp = names.get(i).compareTo(other.names.get(i));
		return cmp;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		return "del " + names.stream().map(Object::toString).collect(Collectors.joining(", "));
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + names.hashCode();
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
		return names.equals(((PyDelName) obj).names);
	}
}
