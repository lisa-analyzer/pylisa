package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.type.PyClassType;

/**
 * Native implementation of {@code object()}: a new object of the root class of
 * the hierarchy (see {@link LibrarySpecificationProvider#hierarchyRoot}), whose
 * methods are not modeled. The name {@code object} is also the type of all the
 * values, e.g. in {@code _sv_verifier.check_type(x, object)}.
 */
public class ObjectBuiltin extends PyNative {

	protected ObjectBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "object", params);
	}

	public static ObjectBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ObjectBuiltin(cfg, location, exprs);
	}

	/**
	 * Whether the values of the given type are objects created by
	 * {@code object()}.
	 *
	 * @param t the type
	 *
	 * @return whether the values are such objects
	 */
	public static boolean isObject(
			Type t) {
		return t.isPointerType() && t.asPointerType().getInnerType().isUnitType()
				&& t.asPointerType().getInnerType().asUnitType()
						.getUnit() == LibrarySpecificationProvider.hierarchyRoot;
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = new DerivedLocation(getLocation(), "object");
		Type type = PyClassType.lookup(LibrarySpecificationProvider.hierarchyRoot.getName());
		ReferenceType reftype = new ReferenceType(type);
		MemoryAllocation alloc = new MemoryAllocation(type, loc, false);
		InstrumentedReceiver object = new InstrumentedReceiver(reftype, false, loc);
		AnalysisState<A> created = analysis.assign(Allocations.allocate(analysis, context, state, alloc, st), object,
				new HeapReference(reftype, alloc, loc), context, st);
		getMetaVariables().add(object);
		return compute(analysis, context, created, object);
	}
}
