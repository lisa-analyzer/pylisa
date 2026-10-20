package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.pylisa.cfg.expression.literal.PyTypeLiteral;

/**
 * A reference to a name that is also the name of a class of the program (e.g.
 * {@code MyClass} in {@code check_type(a, MyClass)}). If the name is not bound
 * to a variable, and it is never assigned in the program (so that it cannot be
 * bound to anything else), its value is the class, as a {@link PyTypeLiteral}.
 * Otherwise, it is a plain variable.
 */
public class PyClassRef extends VariableRef {

	private final ClassUnit unit;

	/**
	 * Builds the reference.
	 *
	 * @param cfg      the cfg where the reference is
	 * @param location the location of the reference
	 * @param unit     the class with the same name
	 */
	public PyClassRef(
			CFG cfg,
			CodeLocation location,
			ClassUnit unit) {
		super(cfg, location, unit.getName());
		this.unit = unit;
	}

	/**
	 * Yields the class with the same name.
	 *
	 * @return the class
	 */
	public ClassUnit getClassUnit() {
		return unit;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		if (entryState.getExecutionState().isBottom() || entryState.knowsIdentifier(getVariable())
				|| PyBuiltinRef.isAssigned(getCFG().getProgram(), getName()))
			return super.forwardSemantics(entryState, interprocedural, expressions);
		return new PyTypeLiteral(getCFG(), getLocation(), unit)
				.forwardSemantics(entryState, interprocedural, expressions);
	}
}
