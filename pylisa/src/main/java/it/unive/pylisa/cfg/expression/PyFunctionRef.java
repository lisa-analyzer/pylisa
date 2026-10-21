package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
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
 * A reference to a name inside a function, that might be the name of a function
 * defined at the top level of the module (e.g. {@code helper} in
 * {@code def f(): return helper}). Global variables are not visible inside
 * functions, so the function would be unknown there. If the name is not bound
 * to a variable of the function, there is a single function with that name in
 * the module, whose {@code def} is executed once (see
 * {@link PyBinaryDispatch#DEFINED_ONCE}), and the name is not assigned anywhere
 * else in the program, its value is that function. Otherwise, it is a plain
 * variable.
 */
public class PyFunctionRef extends VariableRef {

	// for each program, the names assigned somewhere other than by a def
	private static final Map<Program, Set<String>> ASSIGNED = Collections.synchronizedMap(new WeakHashMap<>());

	/**
	 * Builds the reference.
	 *
	 * @param cfg      the cfg where the reference is
	 * @param location the location of the reference
	 * @param name     the name
	 */
	// the unit of the module defining the function (null for the module of
	// the reference), and the name of the function there
	private final String module, member;

	public PyFunctionRef(
			CFG cfg,
			CodeLocation location,
			String name) {
		this(cfg, location, name, null, name);
	}

	/**
	 * Builds a reference to a function of another module (e.g. {@code f} after
	 * {@code from m import f}, or {@code m.f}).
	 *
	 * @param cfg      the cfg where the reference is
	 * @param location the location of the reference
	 * @param name     the name of the reference
	 * @param module   the name of the unit of the module
	 * @param member   the name of the function in the module
	 */
	public PyFunctionRef(
			CFG cfg,
			CodeLocation location,
			String name,
			String module,
			String member) {
		super(cfg, location, name);
		this.module = module;
		this.member = member;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeMember function = entryState.getExecutionState().isBottom() || entryState.knowsIdentifier(getIdentifier())
				? null
				: function();
		if (function == null)
			return super.forwardSemantics(entryState, interprocedural, expressions);
		return new PyFunctionLiteral(getCFG(), getLocation(), function)
				.forwardSemantics(entryState, interprocedural, expressions);
	}

	// the function of the module this reference denotes, if any
	private CodeMember function() {
		Program program = getCFG().getProgram();
		Set<String> assigned = ASSIGNED.computeIfAbsent(program, PyFunctionRef::assigned);
		if (assigned.contains(getName()) || assigned.contains(member))
			return null;
		Unit unit = module == null ? getCFG().getUnit() : program.getUnit(module);
		if (unit == null)
			return null;
		CodeMember found = null;
		for (CodeMember member : unit.getCodeMembersByName(this.member))
			if (member instanceof CFG && member.getDescriptor().getAnnotationList().stream()
					.anyMatch(a -> a.getAnnotationName().equals(PyBinaryDispatch.DEFINED_ONCE))) {
				if (found != null)
					// e.g. two defs with the same name
					return null;
				found = member;
			} else
				return null;
		return found;
	}

	// the names assigned somewhere in the program, other than by the def
	// binding a function to its name
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
		if (st instanceof Assignment && !isDef((Assignment) st))
			targets(((Assignment) st).getLeft(), assigned);
		Expression[] subs = st instanceof NaryExpression ? ((NaryExpression) st).getSubExpressions()
				: st instanceof NaryStatement ? ((NaryStatement) st).getSubExpressions() : new Expression[0];
		for (Expression sub : subs)
			collect(sub, assigned);
	}

	// whether the assignment is the one of def f(...), binding f to the
	// function
	private static boolean isDef(
			Assignment assignment) {
		return assignment.getRight() instanceof PyFunctionLiteral && assignment.getLeft() instanceof VariableRef
				&& ((VariableRef) assignment.getLeft()).getName()
						.equals(((PyFunctionLiteral) assignment.getRight()).getValue().getDescriptor().getName());
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
