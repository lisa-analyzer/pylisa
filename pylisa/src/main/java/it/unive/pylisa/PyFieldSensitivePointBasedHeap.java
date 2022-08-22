package it.unive.pylisa;

import java.lang.reflect.Field;
import java.util.HashSet;
import java.util.Set;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.AllocationSites;
import it.unive.lisa.analysis.heap.pointbased.PointBasedHeap;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.lisa.util.collections.externalSet.ExternalSet;

public class PyFieldSensitivePointBasedHeap extends PointBasedHeap {

	public PyFieldSensitivePointBasedHeap() {
		super();
	}

	private PyFieldSensitivePointBasedHeap(HeapEnvironment<AllocationSites> allocationSites) {
		super(allocationSites);
	}

	@Override
	@SuppressWarnings("unchecked")
	protected PyFieldSensitivePointBasedHeap from(PointBasedHeap original) {
		try {
			Field envField = PointBasedHeap.class.getDeclaredField("heapEnv");
			envField.setAccessible(true);
			HeapEnvironment<AllocationSites> env = (HeapEnvironment<AllocationSites>) envField.get(original);
			return new PyFieldSensitivePointBasedHeap(env);
		} catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e) {
			return new PyFieldSensitivePointBasedHeap();
		}
	}

	@Override
	public ExpressionSet<ValueExpression> rewrite(SymbolicExpression expression, ProgramPoint pp)
			throws SemanticException {
		return expression.accept(new Rewriter());
	}

	private class Rewriter extends PointBasedHeap.Rewriter {

		private Set<ValueExpression> resolveIdentifier(Identifier v) {
			Set<ValueExpression> result = new HashSet<>();
			for (AllocationSite site : heapEnv.getState(v)) {
				MemoryPointer e = new MemoryPointer(
						new ReferenceType(site.getStaticType()),
						site,
						site.getCodeLocation());
				if (v.hasRuntimeTypes())
					e.setRuntimeTypes(v.getRuntimeTypes());
				result.add(e);
			}

			return result;
		}
		
		@Override
		public ExpressionSet<ValueExpression> visit(HeapDereference expression, ExpressionSet<ValueExpression> arg,
				Object... params)
				throws SemanticException {
			Set<ValueExpression> result = new HashSet<>();

			for (ValueExpression ref : arg)
				if (ref instanceof MemoryPointer)
					result.add(((MemoryPointer) ref).getReferencedLocation());
				else if (ref instanceof Identifier) {
					// this could be aliasing!
					Identifier id = (Identifier) ref;
					if (heapEnv.getKeys().contains(id))
						result.addAll(resolveIdentifier(id));
					else if (id instanceof Variable) {
						// this is a variable from the program that we know nothing about
						CodeLocation loc = expression.getCodeLocation();
						AllocationSite site = new AllocationSite(id.getStaticType(), "unknown@" + id.getName(), loc);
						result.add(site);
					}
				} else
					result.add(ref);

			return new ExpressionSet<>(result);
		}

		@Override
		public ExpressionSet<ValueExpression> visit(AccessChild expression, ExpressionSet<ValueExpression> receiver,
				ExpressionSet<ValueExpression> child, Object... params) throws SemanticException {
			Set<ValueExpression> result = new HashSet<>();

			for (ValueExpression rec : receiver)
				if (rec instanceof MemoryPointer) {
					AllocationSite site = (AllocationSite) ((MemoryPointer) rec).getReferencedLocation();
					populate(expression, child, result, site);
				} else if (rec instanceof AllocationSite) {
					AllocationSite site = (AllocationSite) rec;
					populate(expression, child, result, site);
				}

			return new ExpressionSet<>(result);
		}

		protected void populate(AccessChild expression, ExpressionSet<ValueExpression> child,
				Set<ValueExpression> result, AllocationSite site) {
			for (SymbolicExpression target : child) {
				AllocationSite e = new AllocationSite(
						expression.getStaticType(),
						site.getLocationName(),
						target,
						site.isWeak(),
						site.getCodeLocation());
				if (expression.hasRuntimeTypes())
					e.setRuntimeTypes(expression.getRuntimeTypes());
				result.add(e);
			}
		}

		@Override
		public ExpressionSet<ValueExpression> visit(HeapAllocation expression, Object... params)
				throws SemanticException {
			String pp = expression.getCodeLocation().getCodeLocation();

			boolean weak;
			if (alreadyAllocated(pp) != null)
				weak = true;
			else
				weak = false;
			AllocationSite e = new AllocationSite(expression.getStaticType(), pp, weak, expression.getCodeLocation());
			if (expression.hasRuntimeTypes())
				e.setRuntimeTypes(expression.getRuntimeTypes());
			return new ExpressionSet<>(e);
		}

		private AllocationSite alreadyAllocated(String id) {
			for (AllocationSites set : heapEnv.getValues())
				for (AllocationSite site : set)
					if (site.getLocationName().equals(id))
						return site;

			return null;
		}

		@Override
		public ExpressionSet<ValueExpression> visit(PushAny expression, Object... params)
				throws SemanticException {
			if (expression.getStaticType().isPointerType()) {
				ExternalSet<Type> inner = expression.getStaticType().asPointerType().getInnerTypes();

				Type tmp = inner.isEmpty() ? Untyped.INSTANCE
						: inner.reduce(inner.first(), (r, tt) -> r.commonSupertype(tt));
				
				CodeLocation loc = expression.getCodeLocation();
				AllocationSite site = new AllocationSite(tmp, "unknown@" + loc.getCodeLocation(), loc);
				return new ExpressionSet<>(new MemoryPointer(expression.getStaticType(), site, loc));
			}
			return new ExpressionSet<>(expression);
		}
	}
}
