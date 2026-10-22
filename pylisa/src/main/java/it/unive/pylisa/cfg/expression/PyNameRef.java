package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
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
import it.unive.lisa.symbolic.value.GlobalVariable;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.LocalImports;
import it.unive.pylisa.cfg.expression.literal.PyFunctionLiteral;
import it.unive.pylisa.cfg.expression.literal.PyTypeLiteral;
import it.unive.pylisa.scoping.NameKind;
import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.WeakHashMap;
import org.apache.commons.io.FilenameUtils;

/**
 * A reference to a name of the program, resolved following Python's scoping
 * rules. The {@link NameKind} of the name, computed statically by the symbol
 * table of the module, determines the identifier the name resolves to (see
 * {@link #getIdentifier()}): the names of the namespace of the module (i.e. the
 * global ones) are {@link GlobalVariable}s named {@code <module>.<name>}, that
 * are not affected by calls, while the other ones are {@link Variable}s local
 * to the function where they are used.<br>
 * <br>
 * When a global name is read, and it is not known in the analysis state, it
 * might denote something that is not tracked as a value of the namespace of the
 * module. Such cases are resolved statically, in this order:
 * <ol>
 * <li>a name imported from a local module ({@code f} after
 * {@code from m import f}) is resolved as the global {@code f} of
 * {@code m};</li>
 * <li>a class defined in the module is the class itself;</li>
 * <li>a function defined in the module, by a single {@code def} executed once
 * (see {@link PyBinaryDispatch#DEFINED_ONCE}), is the function itself;</li>
 * <li>a builtin function is the function itself.</li>
 * </ol>
 * Otherwise, the name is read as any other global.
 */
public class PyNameRef extends VariableRef {

	// for each program, the names that are assigned somewhere
	private static final Map<Program, Set<String>> ASSIGNED = Collections.synchronizedMap(new WeakHashMap<>());

	/**
	 * Clears the caches of this class, so that sequential analyses in the same
	 * JVM do not interfere with each other.
	 */
	public static void clearAll() {
		ASSIGNED.clear();
	}

	private final NameKind kind;

	// the (dotted) name of the module the name belongs to
	private final String module;

	// the name of the unit of the module the name belongs to
	private final String unit;

	// the member of a local module the name is bound to by an import, if any
	private final LocalImports.Member imported;

	// whether this is the target of a binding (e.g. of an assignment)
	private final boolean target;

	// the text of the reference in the program (e.g. m.f for the global f of
	// module m)
	private final String text;

	/**
	 * Builds the reference.
	 *
	 * @param cfg      the cfg where the reference is
	 * @param location the location of the reference
	 * @param name     the name
	 * @param kind     the kind of the name in the scope of the reference
	 * @param module   the (dotted) name of the module the name belongs to
	 * @param unit     the name of the unit of the module the name belongs to
	 * @param imported the member of a local module the name is bound to by an
	 *                     import ({@code from m import name}), or {@code null}
	 * @param target   whether this is the target of a binding (e.g. of an
	 *                     assignment), that is never resolved statically
	 */
	public PyNameRef(
			CFG cfg,
			CodeLocation location,
			String name,
			NameKind kind,
			String module,
			String unit,
			LocalImports.Member imported,
			boolean target) {
		this(cfg, location, name, kind, module, unit, imported, target, name);
	}

	/**
	 * Builds the reference, that appears in the program with a text different
	 * from the name (e.g. {@code m.f} for the global {@code f} of module
	 * {@code m}).
	 *
	 * @param cfg      the cfg where the reference is
	 * @param location the location of the reference
	 * @param name     the name
	 * @param kind     the kind of the name in the scope of the reference
	 * @param module   the (dotted) name of the module the name belongs to
	 * @param unit     the name of the unit of the module the name belongs to
	 * @param imported the member of a local module the name is bound to by an
	 *                     import ({@code from m import name}), or {@code null}
	 * @param target   whether this is the target of a binding (e.g. of an
	 *                     assignment), that is never resolved statically
	 * @param text     the text of the reference in the program
	 */
	public PyNameRef(
			CFG cfg,
			CodeLocation location,
			String name,
			NameKind kind,
			String module,
			String unit,
			LocalImports.Member imported,
			boolean target,
			String text) {
		super(cfg, location, name);
		this.kind = kind;
		this.module = module;
		this.unit = unit;
		this.imported = imported;
		this.target = target;
		this.text = text;
	}

	@Override
	public String toString() {
		return text;
	}

	/**
	 * Yields the kind of the name in the scope of this reference.
	 *
	 * @return the kind
	 */
	public NameKind getKind() {
		return kind;
	}

	/**
	 * Yields the (dotted) name of the module the name belongs to.
	 *
	 * @return the name of the module
	 */
	public String getModule() {
		return module;
	}

	/**
	 * Yields whether this is the target of a binding (e.g. of an assignment).
	 *
	 * @return whether this is a target
	 */
	public boolean isTarget() {
		return target;
	}

	/**
	 * Yields the identifier a name resolves to: a {@link GlobalVariable} named
	 * {@code <module>.<name>} for the names of the namespace of the module, or
	 * a {@link Variable} otherwise.
	 *
	 * @param kind     the kind of the name
	 * @param module   the (dotted) name of the module the name belongs to
	 * @param name     the name
	 * @param location the location where the name is used
	 *
	 * @return the identifier
	 */
	public static Identifier identifier(
			NameKind kind,
			String module,
			String name,
			CodeLocation location) {
		if (kind.isGlobal())
			return new GlobalVariable(Untyped.INSTANCE, module + "." + name, location);
		return new Variable(Untyped.INSTANCE, name, location);
	}

	@Override
	public Identifier getIdentifier() {
		if (!kind.isGlobal())
			return super.getIdentifier();
		Identifier id = identifier(kind, module, getName(), getLocation());
		for (Annotation ann : getAnnotations())
			id.addAnnotation(ann);
		return id;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		if (target || !kind.isGlobal() || entryState.getExecutionState().isBottom()
				|| entryState.knowsIdentifier(getIdentifier()))
			return super.forwardSemantics(entryState, interprocedural, expressions);

		if (imported != null)
			// the global of the module the name is imported from
			return new PyNameRef(getCFG(), getLocation(), imported.name(), NameKind.GLOBAL_EXPLICIT,
					imported.module(), imported.unit(), null, false)
							.forwardSemantics(entryState, interprocedural, expressions);

		Expression value = staticValue();
		if (value == null)
			return super.forwardSemantics(entryState, interprocedural, expressions);
		return value.forwardSemantics(entryState, interprocedural, expressions);
	}

	// the value of the name when it is not tracked by the analysis, if any
	private Expression staticValue() {
		ClassUnit cls = localClass();
		if (cls != null)
			return new PyTypeLiteral(getCFG(), getLocation(), cls);
		CodeMember function = localFunction();
		if (function != null)
			return new PyFunctionLiteral(getCFG(), getLocation(), function);
		NativeCFG builtin = builtin(getCFG().getProgram(), getName());
		if (builtin != null)
			return new PyFunctionLiteral(getCFG(), getLocation(), builtin);
		return null;
	}

	/**
	 * Yields the class this name denotes statically, if any: a class defined in
	 * the module of the name, or in the local module it is imported from. This
	 * does not take into account the bindings of the name.
	 *
	 * @return the class, or {@code null}
	 */
	public ClassUnit getStaticClass() {
		if (!kind.isGlobal())
			return null;
		if (imported != null)
			return new PyNameRef(getCFG(), getLocation(), imported.name(), NameKind.GLOBAL_EXPLICIT,
					imported.module(), imported.unit(), null, false).getStaticClass();
		return localClass();
	}

	// the class with this name defined in the module, if any
	private ClassUnit localClass() {
		Unit cls = getCFG().getProgram().getUnit(getName());
		if (!(cls instanceof ClassUnit))
			return null;
		CodeLocation location = ((ClassUnit) cls).getLocation();
		if (location instanceof SourceCodeLocation
				&& FilenameUtils.removeExtension(((SourceCodeLocation) location).getSourceFile()).equals(unit))
			return (ClassUnit) cls;
		return null;
	}

	// the function with this name defined in the module by a single def
	// executed once, if any
	private CodeMember localFunction() {
		Unit u = getCFG().getProgram().getUnit(unit);
		if (u == null)
			return null;
		CodeMember found = null;
		for (CodeMember member : u.getCodeMembersByName(getName()))
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
		return ASSIGNED.computeIfAbsent(program, PyNameRef::assigned).contains(name);
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

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + Objects.hash(kind, module, unit, imported, target, text);
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
		PyNameRef other = (PyNameRef) obj;
		return kind == other.kind && target == other.target && Objects.equals(module, other.module)
				&& Objects.equals(unit, other.unit) && Objects.equals(imported, other.imported)
				&& Objects.equals(text, other.text);
	}
}
