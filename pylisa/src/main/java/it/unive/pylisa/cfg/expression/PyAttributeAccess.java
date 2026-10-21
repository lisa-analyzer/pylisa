package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyClassAttributes.ClassAttribute;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.HashSet;
import java.util.Set;

/**
 * An access to the attribute {@code name} of an object ({@code receiver.name}),
 * both when reading it and as the target of an assignment
 * ({@code receiver.name = v}). When the receiver is an object, the attribute is
 * the field {@code name} of the object in the heap, so that its value is
 * tracked across reads and writes (also through aliases, and inside called
 * functions). For any other receiver (e.g. a {@code str}, whose attributes are
 * its methods), the value is unknown.
 */
public class PyAttributeAccess extends UnaryExpression {

	private final String name;

	/**
	 * Builds the access.
	 *
	 * @param cfg      the cfg where the access happens
	 * @param location the location of the access
	 * @param receiver the object whose attribute is accessed
	 * @param name     the name of the attribute
	 */
	public PyAttributeAccess(
			CFG cfg,
			CodeLocation location,
			Expression receiver,
			String name) {
		super(cfg, location, ".", receiver);
		this.name = name;
	}

	/**
	 * Yields the name of the accessed attribute.
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return name.compareTo(((PyAttributeAccess) o).name);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		return read(interprocedural, state, expr, this);
	}

	/**
	 * Reads the attribute from the receiver {@code expr}, raising
	 * {@code AttributeError} from {@code thrower} if it has never been
	 * assigned.
	 *
	 * @param <A>             the kind of the analysis state
	 * @param <D>             the kind of the abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the attribute is read
	 * @param expr            the receiver
	 * @param thrower         the statement raising the errors of the read
	 *
	 * @return the state after the read
	 *
	 * @throws SemanticException if something goes wrong during the computation
	 */
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> read(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			Statement thrower)
			throws SemanticException {
		if (state.getExecutionState().isBottom())
			// unreachable: nothing is computed, and nothing is left on the
			// stack
			return state.withExecution(state.getExecution().bottom());
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();

		Set<Type> objects = new HashSet<>();
		boolean others = false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, expr, this);
		if (types.isEmpty())
			others = true;
		for (Type t : types)
			if (t.isPointerType() && t.asPointerType().getInnerType().isUnitType())
				objects.add(t.asPointerType().getInnerType());
			else
				others = true;

		AnalysisState<A> result = state.bottomExecution();
		// the methods read without calling them: bound methods (see
		// BoundMethods)
		Set<CodeMember> methods = new HashSet<>();
		boolean onlyMethods = !objects.isEmpty() && !isWriteTarget();
		if (onlyMethods)
			for (Type o : objects) {
				Set<CodeMember> m = BoundMethods.methods(getProgram(), new ReferenceType(o), name);
				methods.addAll(m);
				onlyMethods &= !m.isEmpty();
			}
		if (!methods.isEmpty())
			result = result.lub(BoundMethods.create(analysis, state, expr, methods, this, getMetaVariables()));
		if (onlyMethods && !mightBeAssigned(analysis, state, expr))
			// no instance attribute shadows the method
			objects.clear();
		if (!objects.isEmpty()) {
			Missing missing = isWriteTarget() ? null : missing(analysis, state, expr, objects, false);
			if (missing == null || missing.assigned()) {
				// python objects have no declared fields: the attribute is a
				// field created by its first assignment
				HeapDereference container = new HeapDereference(Type.commonSupertype(objects, Untyped.INSTANCE),
						expr, loc);
				Variable field = new Variable(Untyped.INSTANCE, name, new Annotations(), loc);
				result = result.lub(analysis.smallStepSemantics(state,
						new AccessChild(Untyped.INSTANCE, container, field, loc), this));
			}
			if (missing != null)
				// the objects without the attribute: the attribute is looked
				// up in their class
				for (ClassUnit cls : missing.classes())
					result = result.lub(classAttribute(interprocedural, state, cls, thrower));
		}
		if (others) {
			// the methods of builtin values, read without calling them (e.g.
			// "a".upper): bound methods
			Set<CodeMember> valueMethods = new HashSet<>();
			boolean unknown = types.isEmpty() || isWriteTarget();
			if (!unknown)
				for (Type t : types)
					if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType()) {
						Set<CodeMember> m = BoundMethods.methods(getProgram(), t, name);
						valueMethods.addAll(m);
						unknown |= m.isEmpty();
					}
			if (!valueMethods.isEmpty())
				result = result
						.lub(BoundMethods.create(analysis, state, expr, valueMethods, this, getMetaVariables()));
			if (unknown)
				result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), this));
		}
		return result;
	}

	/**
	 * The errors raised by reading this attribute from the objects that
	 * {@code expr} points to whose class does not define it as a method, when
	 * other objects it points to have it as a method (e.g. {@code c.func()},
	 * where {@code c} is an object of a class defining {@code func}, or of one
	 * that does not): that is, the {@code AttributeError} raised for the
	 * objects on which the attribute has never been assigned, and whose class
	 * does not define it as a class attribute.
	 *
	 * @param <A>             the kind of the analysis state
	 * @param <D>             the kind of the abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the attribute is read
	 * @param expr            the receiver
	 * @param thrower         the statement raising the errors of the read
	 *
	 * @return the state with the raised errors, and no normal execution
	 *
	 * @throws SemanticException if something goes wrong during the computation
	 */
	<A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> missingErrors(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			Statement thrower)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> objects = new HashSet<>();
		for (Type t : analysis.getRuntimeTypesOf(state, expr, this))
			if (t.isPointerType() && t.asPointerType().getInnerType().isUnitType())
				objects.add(t.asPointerType().getInnerType());
		Missing missing = objects.isEmpty() ? null : missing(analysis, state, expr, objects, true);
		AnalysisState<A> result = state.bottomExecution();
		if (missing != null)
			for (ClassUnit cls : missing.classes())
				// only the errors: the value of the attribute is not known
				result = result.lub(classAttribute(interprocedural, state, cls, thrower).bottomExecution());
		return result;
	}

	/**
	 * The receiver objects that might not have the attribute.
	 *
	 * @param assigned whether some of the receiver objects have the attribute
	 * @param classes  the classes of the receiver objects that might not have
	 *                     the attribute
	 */
	private record Missing(
			boolean assigned,
			Set<ClassUnit> classes) {
	}

	// whether this access is the target of an assignment (also inside a
	// tuple, as in o.x, y = ...)
	private boolean isWriteTarget() {
		Statement child = this;
		Statement parent = getParentStatement();
		while (parent instanceof TupleCreation) {
			child = parent;
			parent = ((TupleCreation) parent).getParentStatement();
		}
		return parent instanceof Assignment && ((Assignment) parent).getLeft() == child;
	}

	/**
	 * The objects {@code expr} points to that might not have the attribute,
	 * that is, on which the attribute has never been assigned, or {@code null}
	 * if the attribute is surely assigned on all of them (or this is unknown).
	 * This is decided only for objects of classes whose attributes are all
	 * known (that is, classes without bases, where the attribute is not a
	 * method), relying on the fields the heap tracks for each object. Since the
	 * heap tracks the fields assigned on some path, an attribute assigned only
	 * on some paths is considered assigned.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Missing missing(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression expr,
			Set<Type> objects,
			boolean skipMethods)
			throws SemanticException {
		if (name.startsWith("__") && name.endsWith("__"))
			// special attributes (e.g. __class__, __dict__) are defined for
			// all objects
			return null;
		Set<ClassUnit> units = new HashSet<>();
		for (Type t : objects) {
			if (!PyBinaryDispatch.fullyKnown(t) || !(t.asUnitType().getUnit() instanceof ClassUnit))
				return null;
			ClassUnit unit = (ClassUnit) t.asUnitType().getUnit();
			if (PyClassAttributes.isMethod(unit, name))
				if (skipMethods)
					// the objects of this class have the attribute
					continue;
				else
					return null;
			units.add(unit);
		}
		if (units.isEmpty())
			return null;

		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (heap == null || heap.isTop() || heap.fields.isTop())
			return null;

		boolean assigned = false;
		Set<ClassUnit> missing = new HashSet<>();
		for (SymbolicExpression rec : analysis.rewrite(state, expr, this)) {
			// the objects the receiver points to
			Set<AllocationSite> sites;
			if (rec instanceof MemoryPointer && ((MemoryPointer) rec).getReferencedLocation() instanceof AllocationSite)
				sites = Set.of((AllocationSite) ((MemoryPointer) rec).getReferencedLocation());
			else if (rec instanceof Identifier && heap.knowsIdentifier((Identifier) rec)) {
				// a variable, or a field of another object
				AllocationSites pointed = heap.getState((Identifier) rec);
				if (pointed.isTop() || pointed.isBottom())
					return null;
				sites = pointed.elements();
			} else if (skipMethods) {
				// the objects are not known (e.g. the receiver is a field
				// whose objects the heap does not track): the receiver might
				// be any object of the classes without the method, that are
				// the ones the heap tracks (if any)
				sites = new HashSet<>();
				if (heap.fields.function != null)
					for (AllocationSite site : heap.fields.getKeys())
						if (site.getStaticType().isUnitType()
								&& units.contains(site.getStaticType().asUnitType().getUnit()))
							sites.add(site);
				if (sites.isEmpty())
					missing.addAll(units);
			} else
				return null;

			for (AllocationSite site : sites) {
				ExpressionSet fields = heap.fields.getState(site);
				if (fields.isTop())
					return null;
				if (fields.elements().stream()
						.anyMatch(f -> f instanceof Variable && ((Variable) f).getName().equals(name)))
					assigned = true;
				else {
					// the class of the object, if known
					Type type = site.getStaticType();
					if (type.isUnitType() && units.contains(type.asUnitType().getUnit()))
						missing.add((ClassUnit) type.asUnitType().getUnit());
					else if (!type.isUnitType())
						missing.addAll(units);
					// otherwise, an object of a class defining the method, or
					// of a class that the types of the receiver exclude (e.g.
					// after isinstance(x, C)), that x cannot point to
				}
			}
		}
		return missing.isEmpty() ? null : new Missing(assigned, missing);
	}

	// whether some of the objects expr points to might have the attribute
	// (that is, it is assigned on them, or the objects are not known)
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean mightBeAssigned(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression expr)
			throws SemanticException {
		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (heap == null || heap.isTop() || heap.fields.isTop() || heap.fields.function == null)
			return true;
		for (SymbolicExpression rec : analysis.rewrite(state, expr, this)) {
			Set<AllocationSite> sites;
			if (rec instanceof MemoryPointer && ((MemoryPointer) rec).getReferencedLocation() instanceof AllocationSite)
				sites = Set.of((AllocationSite) ((MemoryPointer) rec).getReferencedLocation());
			else if (rec instanceof Identifier && heap.knowsIdentifier((Identifier) rec)) {
				AllocationSites pointed = heap.getState((Identifier) rec);
				if (pointed.isTop() || pointed.isBottom())
					return true;
				sites = pointed.elements();
			} else
				return true;
			for (AllocationSite site : sites) {
				ExpressionSet fields = heap.fields.getState(site);
				if (fields.isTop() || fields.elements().stream()
						.anyMatch(f -> f instanceof Variable && ((Variable) f).getName().equals(name)))
					return true;
			}
		}
		return false;
	}

	// reads the attribute from the given class, for an object that does not
	// have it: an attribute that the class does not define raises
	// AttributeError
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> classAttribute(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ClassUnit cls,
			Statement thrower)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ClassAttribute attribute = PyClassAttributes.attribute(cls, name);
		if (attribute == null)
			return PyExceptions.raise(analysis, state, getCFG(), getLocation(), thrower,
					LibrarySpecificationProvider.ATTRIBUTE_ERROR);
		if (attribute.value() == null)
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, getLocation()), this);
		// the body of the class is not executed: its constant value is
		// computed here
		Expression value = attribute.value();
		AnalysisState<A> result = value.forwardSemantics(state, interprocedural, new StatementStore<>(state));
		// e.g. the return value of the call computing -1: they are forgotten
		// by the statement using this attribute
		getMetaVariables().addAll(value.getMetaVariables());
		return result;
	}

	@Override
	public String toString() {
		return getSubExpression() + "." + name;
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + name.hashCode();
	}

	@Override
	public boolean equals(
			Object obj) {
		return super.equals(obj) && name.equals(((PyAttributeAccess) obj).name);
	}
}
