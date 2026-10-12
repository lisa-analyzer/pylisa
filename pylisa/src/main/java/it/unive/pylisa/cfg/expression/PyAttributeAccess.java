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
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.PyClassParser;
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

		AnalysisState<A> result = state.bottom();
		if (!objects.isEmpty()) {
			Assigned assigned = isWriteTarget() ? Assigned.SURELY : assigned(analysis, state, expr, objects);
			if (assigned != Assigned.NEVER) {
				// python objects have no declared fields: the attribute is a
				// field created by its first assignment
				HeapDereference container = new HeapDereference(Type.commonSupertype(objects, Untyped.INSTANCE),
						expr, loc);
				Variable field = new Variable(Untyped.INSTANCE, name, new Annotations(), loc);
				result = result.lub(analysis.smallStepSemantics(state,
						new AccessChild(Untyped.INSTANCE, container, field, loc), this));
			}
			if (assigned != Assigned.SURELY)
				result = result.lub(PyExceptions.raise(analysis, state, getCFG(), loc, this,
						LibrarySpecificationProvider.ATTRIBUTE_ERROR));
		}
		if (others)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), this));
		return result;
	}

	// whether the attribute has been assigned on the receiver objects
	private enum Assigned {
		// on all of them, or it is unknown
		SURELY,
		// on some of them only
		SOMETIMES,
		// on none of them
		NEVER
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
	 * Whether the attribute has been assigned on the objects {@code expr}
	 * points to. Reading an attribute that has never been assigned raises
	 * {@code AttributeError}: this is decided only for objects of classes whose
	 * attributes are all known (that is, classes without bases, where the
	 * attribute is not a method nor a class attribute), relying on the fields
	 * the heap tracks for each object. Since the heap tracks the fields
	 * assigned on some path, an attribute assigned only on some paths is
	 * considered assigned.
	 */
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Assigned assigned(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression expr,
			Set<Type> objects)
			throws SemanticException {
		if (name.startsWith("__") && name.endsWith("__"))
			// special attributes (e.g. __class__, __dict__) are defined for
			// all objects
			return Assigned.SURELY;
		for (Type t : objects)
			if (!PyBinaryDispatch.fullyKnown(t) || definedByClass((ClassUnit) t.asUnitType().getUnit()))
				return Assigned.SURELY;

		HeapEnvWithFields heap = state.getExecutionState().getLatticeInstance(HeapEnvWithFields.class);
		if (heap == null || heap.isTop() || heap.fields.isTop())
			return Assigned.SURELY;

		boolean assigned = false, missing = false;
		for (SymbolicExpression rec : analysis.rewrite(state, expr, this)) {
			// the objects the receiver points to
			Set<AllocationSite> sites;
			if (rec instanceof MemoryPointer && ((MemoryPointer) rec).getReferencedLocation() instanceof AllocationSite)
				sites = Set.of((AllocationSite) ((MemoryPointer) rec).getReferencedLocation());
			else if (rec instanceof Identifier && heap.knowsIdentifier((Identifier) rec)) {
				// a variable, or a field of another object
				AllocationSites pointed = heap.getState((Identifier) rec);
				if (pointed.isTop() || pointed.isBottom())
					return Assigned.SURELY;
				sites = pointed.elements();
			} else
				return Assigned.SURELY;

			for (AllocationSite site : sites) {
				ExpressionSet fields = heap.fields.getState(site);
				if (fields.isTop())
					return Assigned.SURELY;
				if (fields.elements().stream()
						.anyMatch(f -> f instanceof Variable && ((Variable) f).getName().equals(name)))
					assigned = true;
				else
					missing = true;
			}
		}
		return !missing ? Assigned.SURELY : assigned ? Assigned.SOMETIMES : Assigned.NEVER;
	}

	// whether the attribute is a method or a class attribute of the given
	// class
	private boolean definedByClass(
			ClassUnit unit) {
		if (!unit.getInstanceCodeMembersByName(name, true).isEmpty() || !unit.getCodeMembersByName(name).isEmpty())
			return true;
		for (CodeMember body : unit.getCodeMembersByName(PyClassParser.INSTRUMENTED_CLASS_INIT_NAME))
			if (body instanceof CFG)
				for (Statement st : ((CFG) body).getNodes())
					if (st instanceof Assignment && ((Assignment) st).getLeft() instanceof VariableRef
							&& ((VariableRef) ((Assignment) st).getLeft()).getName().equals(name))
						return true;
		return false;
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
