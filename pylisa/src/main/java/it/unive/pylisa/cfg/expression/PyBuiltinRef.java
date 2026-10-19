package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.pylisa.cfg.expression.literal.PyFunctionLiteral;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

/**
 * A reference to a name that is also the name of a builtin function of the
 * library (e.g. {@code len} in {@code q = len}). If the name is not bound to a
 * variable, and it is never assigned in the program (so that it cannot be bound
 * to anything else), its value is the builtin function, as for the functions
 * bound by {@code def} statements (see {@link PyFunctionLiteral}): calls
 * through variables holding it are then resolved (see {@link PyNameCall}).
 * Otherwise, it is a plain variable.
 */
public class PyBuiltinRef extends VariableRef {

	// for each program, the names that are assigned somewhere
	private static final Map<Program, Set<String>> ASSIGNED = Collections.synchronizedMap(new WeakHashMap<>());

	private final NativeCFG function;

	/**
	 * Builds the reference.
	 *
	 * @param cfg      the cfg where the reference is
	 * @param location the location of the reference
	 * @param function the builtin function with the same name
	 */
	public PyBuiltinRef(
			CFG cfg,
			CodeLocation location,
			NativeCFG function) {
		super(cfg, location, function.getDescriptor().getName());
		this.function = function;
	}

	/**
	 * Yields the builtin function named {@code name}, if any.
	 *
	 * @param program the program
	 * @param name    the name
	 *
	 * @return the function, or {@code null}
	 */
	public static NativeCFG builtin(
			Program program,
			String name) {
		for (CodeMember member : program.getCodeMembersByName(name))
			if (member instanceof NativeCFG)
				return (NativeCFG) member;
		return null;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		if (entryState.getExecutionState().isBottom() || entryState.knowsIdentifier(getVariable())
				|| isAssigned(getCFG().getProgram(), getName()))
			return super.forwardSemantics(entryState, interprocedural, expressions);
		return new PyFunctionLiteral(getCFG(), getLocation(), function)
				.forwardSemantics(entryState, interprocedural, expressions);
	}

	/**
	 * Whether {@code name} is assigned somewhere in the program (also inside
	 * tuples, as in {@code a, b = ...}).
	 *
	 * @param program the program
	 * @param name    the name
	 *
	 * @return whether the name is assigned
	 */
	public static boolean isAssigned(
			Program program,
			String name) {
		return ASSIGNED.computeIfAbsent(program, PyBuiltinRef::assigned).contains(name);
	}

	// the names assigned somewhere in the program
	private static Set<String> assigned(
			Program program) {
		Set<String> assigned = new HashSet<>();
		for (CFG cfg : program.getAllCFGs())
			for (Statement st : cfg.getNodes())
				collect(st, assigned);
		return assigned;
	}

	private static void collect(
			Statement st,
			Set<String> assigned) {
		if (st instanceof Assignment)
			targets(((Assignment) st).getLeft(), assigned);
		Expression[] subs = st instanceof NaryExpression ? ((NaryExpression) st).getSubExpressions()
				: st instanceof NaryStatement ? ((NaryStatement) st).getSubExpressions() : new Expression[0];
		for (Expression sub : subs)
			collect(sub, assigned);
	}

	private static void targets(
			Expression target,
			Set<String> assigned) {
		if (target instanceof VariableRef)
			assigned.add(((VariableRef) target).getName());
		else if (target instanceof TupleCreation)
			for (Expression e : ((TupleCreation) target).getSubExpressions())
				targets(e, assigned);
	}
}
