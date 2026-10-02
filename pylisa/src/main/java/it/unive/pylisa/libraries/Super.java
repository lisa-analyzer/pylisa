package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.TypeConv;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import it.unive.pylisa.program.PyClassUnit;
import java.util.HashSet;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Pluggable semantics for Python's built-in {@code super(cls, self)} call.
 * <p>
 * Receives two arguments from the call site:
 * <ol>
 * <li>{@code cls} — a reference to the enclosing class (the first arg to
 * {@code super()})</li>
 * <li>{@code self} — the receiver object</li>
 * </ol>
 * <p>
 * The semantics type-convert {@code self} to the first known
 * {@link ClassUnit}/{@link PyClassUnit} ancestor of {@code cls} in the MRO,
 * falling back to {@code builtins.object} if no suitable ancestor is found.
 */
public class Super extends it.unive.lisa.program.cfg.statement.BinaryExpression implements PluggableStatement {

	private static final Logger log = LogManager.getLogger(Super.class);

	/** The originating call statement, used as the semantic proxy node. */
	protected Statement st;

	/**
	 * Constructs a Super expression.
	 *
	 * @param cfg           the CFG this expression belongs to
	 * @param location      source location
	 * @param constructName the construct name (always {@code "super"})
	 * @param expressions   the two argument expressions: {@code [cls, self]}
	 */
	protected Super(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression[] expressions) {
		super(cfg, location, constructName, expressions[0], expressions[1]);
	}

	/**
	 * Factory method required by {@link PluggableStatement} reflective
	 * instantiation.
	 *
	 * @param cfg      the CFG
	 * @param location the source location
	 * @param exprs    the arguments: {@code [cls, self]}
	 *
	 * @return a new {@link Super} expression
	 */
	public static Super build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		log.debug("Super.build: exprs[0]={} ({}) id={}, exprs[1]={} id={}",
				exprs[0], exprs[0].getClass().getSimpleName(),
				System.identityHashCode(exprs[0]),
				exprs[1], System.identityHashCode(exprs[1]));
		return new Super(cfg, location, "super", exprs);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		return "super";
	}

	@Override
	public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	/**
	 * Implements the abstract semantics of {@code super(cls, self)}.
	 * <p>
	 * Resolves the parent class by inspecting the runtime types of the
	 * {@code cls} argument. For each resolved {@link PyClassType}, the first
	 * ancestor that is a {@link ClassUnit} or {@link PyClassUnit} is selected
	 * as the super-class. If none is found, falls back to
	 * {@code builtins.object}. The result is a type-conversion of {@code self}
	 * to the resolved parent type.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the current abstract state
	 * @param left            the evaluated {@code cls} argument
	 * @param right           the evaluated {@code self} argument
	 * @param expressions     the statement store
	 *
	 * @return the abstract state after the type conversion
	 */
	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();

		Set<Type> clsTypes = analysis.getRuntimeTypesOf(state, left, this);
		log.debug("super(): fwdBinarySemantics called, left={}, clsTypes={}", left, clsTypes);

		AnalysisState<A> result = state.bottomExecution();
		boolean anyResolved = false;

		for (Type t : clsTypes) {
			if (!(t instanceof PyClassType classType))
				continue;

			CompilationUnit parentUnit = resolveParentClass(classType, analysis);
			if (parentUnit == null)
				continue;

			if (!PyClassType.isRegistered(parentUnit.getName())) {
				log.warn("super(): parent class '{}' has no registered PyClassType; skipping", parentUnit.getName());
				continue;
			}

			PyClassType parentClassType = PyClassType.lookup(parentUnit.getName());
			ReferenceType refTypeTo = new ReferenceType(parentClassType);

			HashSet<Type> tokenTypes = new HashSet<>();
			tokenTypes.add(refTypeTo);
			PyTypeTokenType tokenTypeTo = new PyTypeTokenType(tokenTypes);

			BinaryExpression be = new BinaryExpression(
					refTypeTo,
					right,
					new Constant(tokenTypeTo, parentUnit, getLocation()),
					TypeConv.INSTANCE,
					getLocation());
			result = result.lub(analysis.smallStepSemantics(state, be, this));
			anyResolved = true;
		}

		if (anyResolved)
			return result;

		// No class type resolved: fall back to returning the self reference
		// unchanged to preserve analysis precision rather than going to top.
		log.warn("super(): could not resolve any class type from left argument; returning self unchanged");
		return analysis.smallStepSemantics(state, right, this);
	}

	/**
	 * Returns the first ancestor of {@code classType}'s unit that is a
	 * {@link ClassUnit} or {@link PyClassUnit}, in immediate-ancestor order.
	 * Falls back to {@code builtins.object} if the class has no such ancestor
	 * or if the unit cannot be found.
	 *
	 * @param classType the class whose parent is sought
	 * @param analysis  the analysis (used only for logging context)
	 *
	 * @return the parent {@link CompilationUnit}, or {@code null} if even
	 *             {@code builtins.object} is unavailable
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> CompilationUnit resolveParentClass(
			PyClassType classType,
			Analysis<A, D> analysis) {
		CompilationUnit unit = classType.getUnit();
		if (unit == null) {
			log.warn("super(): PyClassType '{}' has no associated CompilationUnit", classType);
			return fallbackToObject();
		}

		log.debug("super(): resolveParentClass for '{}', ancestors={}", unit.getName(), unit.getImmediateAncestors());
		for (CompilationUnit ancestor : unit.getImmediateAncestors()) {
			if (ancestor instanceof ClassUnit || ancestor instanceof PyClassUnit)
				return ancestor;
		}

		log.debug("super(): class '{}' has no ClassUnit/PyClassUnit ancestor; falling back to builtins.object",
				unit.getName());
		return fallbackToObject();
	}

	/**
	 * Returns the {@code builtins.object} compilation unit, or {@code null} if
	 * it has not been registered.
	 *
	 * @return the {@code builtins.object} unit, or {@code null}
	 */
	private CompilationUnit fallbackToObject() {
		if (!PyClassType.isRegistered("builtins.object")) {
			log.warn("super(): builtins.object is not registered; cannot resolve parent class");
			return null;
		}
		return PyClassType.lookup("builtins.object").getUnit();
	}
}
