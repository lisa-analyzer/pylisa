package it.unive.pylisa.analysis.heap;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.GenericMapLattice;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.lattices.heap.allocations.AllocationSite;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.heap.allocations.HeapAllocationSite;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.lisa.util.datastructures.trie.PatriciaTrieMap;
import it.unive.pylisa.cfg.ClassBases;
import it.unive.pylisa.cfg.expression.PyAttributeAccess;
import it.unive.pylisa.libraries.TypeHints;
import it.unive.pylisa.symbolic.operators.compare.IsInstance;
import it.unive.pylisa.symbolic.operators.compare.NotIsInstance;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import java.util.WeakHashMap;
import org.apache.commons.lang3.tuple.Pair;

/**
 * The field-sensitive point-based heap, where reading a field holding objects
 * yields them (see {@link #process}), also refined by {@code isinstance} (see
 * {@link IsInstance}): in the branches of {@code isinstance(x, C)}, {@code x}
 * points only to the objects whose class satisfies {@code C}, or only to the
 * ones whose class does not, as its types are refined by the type domain (see
 * {@code PyInferredTypes}). Otherwise, the methods called on {@code x} would
 * also see the objects that its types exclude.
 */
public class PyHeap extends FieldSensitivePointBasedHeap {

	/**
	 * Assigns {@code rhs} to {@code id}. Reading a field holding objects (e.g.
	 * {@code o.x} in {@code y = o.x}) yields the location of the field: the
	 * identifier then points to the objects the field points to (otherwise it
	 * would point to nothing, as only pointers are stored).
	 */
	@Override
	protected HeapEnvWithFields process(
			HeapEnvWithFields state,
			Identifier id,
			SymbolicExpression rhs,
			ProgramPoint pp,
			SemanticOracle oracle,
			List<HeapReplacement> replacements,
			boolean rhsIsReceiver)
			throws SemanticException {
		// TODO move to LiSA: AllocationSiteBasedAnalysis.assign follows the
		// references held by the right-hand side only when the assigned
		// expression is itself an identifier (x = y). When it is a field access
		// (x = o.f), it is rewritten to the heap location of the field, that
		// process ignores as it is not a MemoryPointer: the assigned identifier
		// then references nothing, losing the object (reads through it, as x.v,
		// yield unknown values, or nothing for identifiers without the
		// fallback of rewriteHeapDereference). Here, a location that is tracked
		// by the heap (i.e. it holds references) is resolved to the
		// references it holds, as done for identifiers, and each of them is
		// processed as the right-hand side of the assignment.
		if (!(rhs instanceof AllocationSite) || !state.knowsIdentifier((Identifier) rhs))
			return super.process(state, id, rhs, pp, oracle, replacements, rhsIsReceiver);
		HeapEnvWithFields result = state.bottom();
		for (SymbolicExpression pointer : resolveIdentifier(state, (Identifier) rhs, pp))
			result = result.lub(super.process(state, id, pointer, pp, oracle, replacements, rhsIsReceiver));
		return result;
	}

	@Override
	public Pair<HeapEnvWithFields, List<HeapReplacement>> assume(
			HeapEnvWithFields state,
			SymbolicExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		Pair<HeapEnvWithFields, List<HeapReplacement>> result = super.assume(state, expression, src, dest, oracle);
		HeapEnvWithFields assumed = result.getLeft();
		// the expression is not rewritten yet (e.g. not o.x), so negations
		// are removed here
		SymbolicExpression test = expression;
		boolean negated = false;
		while (test instanceof UnaryExpression
				&& ((UnaryExpression) test).getOperator() == LogicalNegation.INSTANCE) {
			negated = !negated;
			test = ((UnaryExpression) test).getExpression();
		}
		if (assumed.isBottom() || assumed.isTop() || !(test instanceof BinaryExpression))
			return result;
		BinaryExpression binary = (BinaryExpression) test;
		if ((binary.getOperator() != IsInstance.INSTANCE && binary.getOperator() != NotIsInstance.INSTANCE)
				|| !(binary.getLeft() instanceof Identifier))
			return result;
		boolean positive = (binary.getOperator() == IsInstance.INSTANCE) != negated;
		Identifier id = (Identifier) binary.getLeft();
		if (!assumed.knowsIdentifier(id))
			return result;
		AllocationSites sites = assumed.getState(id);
		Set<Type> hints = oracle.getRuntimeTypesOf(binary.getRight(), src);
		if (sites.isTop() || sites.isBottom() || hints.isEmpty())
			return result;

		Set<AllocationSite> kept = new HashSet<>();
		for (AllocationSite site : sites.elements()) {
			Type object = new ReferenceType(site.getStaticType());
			for (Type hint : hints)
				if (TypeHints.isType(hint) != Satisfiability.NOT_SATISFIED && TypeHints.satisfies(object,
						hint) != (positive ? Satisfiability.NOT_SATISFIED : Satisfiability.SATISFIED)) {
					kept.add(site);
					break;
				}
		}
		if (kept.size() == sites.elements().size())
			return result;
		if (kept.isEmpty())
			return Pair.of(assumed.bottom(), result.getRight());
		return Pair.of(assumed.putState(id, new AllocationSites(kept)), result.getRight());
	}

	/**
	 * The prefix of the name of the field recording, in the fields of an
	 * allocation site, that an attribute of its objects might not be
	 * initialized (see {@link #surelyInitialized}).
	 */
	public static final String UNINITIALIZED = "$uninitialized:";

	// the names of the attributes assigned by the programs (see attributes)
	private static final Map<Program, Set<String>> ATTRIBUTES = Collections.synchronizedMap(new WeakHashMap<>());

	/**
	 * Yields whether the field of the objects of the given allocation site is
	 * surely initialized: the allocation of an object of a class of the program
	 * records that each attribute assigned by the program might not be
	 * initialized, and a strong assignment of the attribute (that is, on a
	 * strong site) removes the record. As the fields of the sites are joined by
	 * union, the record survives a join if the attribute is not assigned on one
	 * of the joined paths. A field is considered initialized if the fields of
	 * the site are not known.
	 *
	 * @param state the heap
	 * @param site  the allocation site (without field)
	 * @param field the name of the field
	 *
	 * @return whether the field is surely initialized
	 */
	public static boolean surelyInitialized(
			HeapEnvWithFields state,
			AllocationSite site,
			String field) {
		if (state.fields.isTop() || state.fields.isBottom() || state.fields.function == null)
			return true;
		ExpressionSet fields = state.fields.getState(site);
		return fields.isTop() || !fields.elements().contains(uninitialized(field, site.getCodeLocation()));
	}

	private static Variable uninitialized(
			String field,
			CodeLocation loc) {
		return new Variable(Untyped.INSTANCE, UNINITIALIZED + field, loc);
	}

	// the names of the attributes that the program assigns (o.x = ...)
	private static Set<String> attributes(
			Program program) {
		return ATTRIBUTES.computeIfAbsent(program, p -> {
			Set<String> names = new TreeSet<>();
			GraphVisitor<CFG, Statement, Edge, Void> visitor = new GraphVisitor<>() {
				@Override
				public boolean visit(
						Void tool,
						CFG graph,
						Statement node) {
					if (node instanceof PyAttributeAccess && ((PyAttributeAccess) node).isWriteTarget())
						names.add(((PyAttributeAccess) node).getName());
					return true;
				}
			};
			for (CFG cfg : p.getAllCFGs())
				for (Statement node : cfg.getNodes())
					node.accept(visitor, null);
			return names;
		});
	}

	// whether the allocated objects are objects of a class of the program
	private static boolean programObject(
			MemoryAllocation allocation) {
		Type type = allocation.getStaticType();
		return type.isUnitType() && ClassBases.isProgramClass(type.asUnitType().getUnit());
	}

	@Override
	public Pair<HeapEnvWithFields, List<HeapReplacement>> smallStepSemantics(
			HeapEnvWithFields state,
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Pair<HeapEnvWithFields, List<HeapReplacement>> result = super.smallStepSemantics(state, expression, pp,
				oracle);
		HeapEnvWithFields st = result.getLeft();
		// (when the fields of all the objects are not known yet, they become
		// known for the new object, as done when accessing a field)
		if (!(expression instanceof MemoryAllocation) || !programObject((MemoryAllocation) expression)
				|| st.isBottom() || st.fields.isBottom())
			return result;
		// the attributes of the new object might not be initialized
		Set<String> names = attributes(pp.getProgram());
		if (names.isEmpty())
			return result;
		PatriciaTrieMap<AllocationSite, ExpressionSet> mapping = st.fields.mkNewFunction(st.fields.function, false);
		for (SymbolicExpression site : rewriteMemoryAllocation((MemoryAllocation) expression, st, pp, oracle))
			for (String name : names)
				mapping = addField((AllocationSite) site, uninitialized(name, site.getCodeLocation()), mapping);
		return Pair.of(new HeapEnvWithFields(st.lattice, st.function, new GenericMapLattice<>(st.fields.lattice,
				mapping)), result.getRight());
	}

	/**
	 * Assigns {@code expression} to {@code id}: if {@code id} is a field of the
	 * objects of a strong allocation site, the field becomes surely initialized
	 * (see {@link #surelyInitialized}).
	 */
	@Override
	public Pair<HeapEnvWithFields, List<HeapReplacement>> assign(
			HeapEnvWithFields state,
			Identifier id,
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Pair<HeapEnvWithFields, List<HeapReplacement>> result = super.assign(state, id, expression, pp, oracle);
		HeapEnvWithFields st = result.getLeft();
		if (!(id instanceof AllocationSite) || ((AllocationSite) id).getField() == null || id.isWeak()
				|| st.isBottom() || st.fields.isTop() || st.fields.isBottom() || st.fields.function == null)
			return result;
		AllocationSite field = (AllocationSite) id;
		AllocationSite site = field.withoutField();
		ExpressionSet fields = st.fields.getState(site);
		Variable record = uninitialized(field.getField(), site.getCodeLocation());
		if (fields.isTop() || !fields.elements().contains(record))
			return result;
		Set<SymbolicExpression> remaining = new HashSet<>(fields.elements());
		remaining.remove(record);
		PatriciaTrieMap<AllocationSite, ExpressionSet> mapping = st.fields.mkNewFunction(st.fields.function, false)
				.put(site, new ExpressionSet(remaining));
		return Pair.of(new HeapEnvWithFields(st.lattice, st.function, new GenericMapLattice<>(st.fields.lattice,
				mapping)), result.getRight());
	}

	@Override
	public ExpressionSet rewriteMemoryAllocation(
			MemoryAllocation expression,
			HeapEnvWithFields state,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return sites(super.rewriteMemoryAllocation(expression, state, pp, oracle), state);
	}

	@Override
	public ExpressionSet rewriteAccessChild(
			AccessChild expression,
			ExpressionSet receiver,
			ExpressionSet child,
			HeapEnvWithFields state,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return sites(super.rewriteAccessChild(expression, receiver, child, state, pp, oracle), state);
	}

	@Override
	public ExpressionSet rewriteHeapDereference(
			HeapDereference expression,
			ExpressionSet arg,
			HeapEnvWithFields state,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return sites(super.rewriteHeapDereference(expression, arg, state, pp, oracle), state);
	}

	@Override
	public ExpressionSet rewritePushAny(
			PushAny expression,
			HeapEnvWithFields state,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return sites(super.rewritePushAny(expression, state, pp, oracle), state);
	}

	// the given expressions, where the heap allocation sites (also the ones
	// referenced by memory pointers) are allocation sites of pylisa, with the
	// initialization of their fields
	private static ExpressionSet sites(
			ExpressionSet expressions,
			HeapEnvWithFields state) {
		Set<SymbolicExpression> result = new HashSet<>();
		for (SymbolicExpression e : expressions)
			if (e instanceof MemoryPointer && ((MemoryPointer) e).getReferencedLocation() instanceof AllocationSite) {
				MemoryPointer pointer = (MemoryPointer) e;
				AllocationSite site = (AllocationSite) pointer.getReferencedLocation();
				result.add(site instanceof HeapAllocationSite && !(site instanceof PyAllocationSite)
						? new MemoryPointer(pointer.getStaticType(), site(site, state), pointer.getAnnotations(),
								pointer.getCodeLocation())
						: pointer);
			} else if (e instanceof HeapAllocationSite && !(e instanceof PyAllocationSite))
				result.add(site((AllocationSite) e, state));
			else
				result.add(e);
		return new ExpressionSet(result);
	}

	private static PyAllocationSite site(
			AllocationSite site,
			HeapEnvWithFields state) {
		return PyAllocationSite.of(site,
				site.getField() == null || surelyInitialized(state, site.withoutField(), site.getField()));
	}
}
