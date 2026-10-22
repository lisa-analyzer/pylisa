package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.List;
import java.util.stream.Collectors;

/**
 * The creation of the closure of a nested function (or lambda) that refers to
 * variables of the enclosing functions, i.e. the value of the function (see
 * {@link it.unive.pylisa.cfg.type.PyClosureUnit}): an object with a field for
 * each free variable, pointing to the object holding the cell of the variable.
 * The cell is owned either by the function (or comprehension) where the closure
 * is created, or by one of its enclosing functions, reached through the closure
 * of the function where the closure is created (see {@link Cells}).
 */
public class PyClosureCreation extends Expression {

	/**
	 * A free variable of the function, and where its cell is.
	 *
	 * @param name  the name of the variable
	 * @param cells the name of the local variable referencing the object
	 *                  holding the cell, or {@code null} if the cell is reached
	 *                  through the closure of the function where the closure is
	 *                  created
	 */
	public record FreeVariable(
			String name,
			String cells) {
	}

	private final PyClassType type;

	private final CFG function;

	private final List<FreeVariable> free;

	/**
	 * Builds the expression.
	 *
	 * @param cfg      the cfg where the closure is created
	 * @param location the location of the creation
	 * @param type     the type of the closure
	 * @param function the function of the closure
	 * @param free     the free variables of the function
	 */
	public PyClosureCreation(
			CFG cfg,
			CodeLocation location,
			PyClassType type,
			CFG function,
			List<FreeVariable> free) {
		super(cfg, location, type.getReference());
		this.type = type;
		this.function = function;
		this.free = free;
	}

	/**
	 * Yields the function of the closure.
	 *
	 * @return the function
	 */
	public CFG getFunction() {
		return function;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		// the closure is held by a meta variable, until it is consumed
		String name = "$closure@" + loc;
		Variable closure = new Variable(Untyped.INSTANCE, name, loc);
		getMetaVariables().add(closure);
		AnalysisState<A> state = Cells.allocate(analysis, entryState, type, name, loc, this);
		for (FreeVariable var : free) {
			// the object holding the cell (note that copying $closure.x into
			// the field of the new closure relies on the heap following the
			// references held by fields, see PyHeap)
			SymbolicExpression owner = var.cells() != null
					? new Variable(Untyped.INSTANCE, var.cells(), loc)
					: Cells.cell(Cells.CLOSURE, var.name(), loc);
			AnalysisState<A> evaluated = analysis.smallStepSemantics(state, owner, this);
			if (evaluated.getExecutionState().isBottom() && !state.getExecutionState().isBottom())
				// the object holding the cell is not known
				evaluated = analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), this);
			AnalysisState<A> assigned = state.bottomExecution();
			for (SymbolicExpression value : evaluated.getExecutionExpressions())
				assigned = assigned.lub(analysis.assign(evaluated, Cells.cell(name, var.name(), loc), value, this));
			state = assigned;
		}
		return analysis.smallStepSemantics(state, closure, this);
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		PyClosureCreation other = (PyClosureCreation) o;
		int cmp = type.toString().compareTo(other.type.toString());
		return cmp != 0 ? cmp : free.toString().compareTo(other.free.toString());
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		return "<closure " + function.getDescriptor().getName() + " of "
				+ free.stream().map(FreeVariable::name).collect(Collectors.joining(", ")) + ">";
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + type.hashCode() + free.hashCode();
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
		PyClosureCreation other = (PyClosureCreation) obj;
		return type.equals(other.type) && free.equals(other.free);
	}
}
