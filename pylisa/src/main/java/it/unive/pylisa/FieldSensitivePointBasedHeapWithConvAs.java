package it.unive.pylisa;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.*;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.binary.TypeConv;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

public class FieldSensitivePointBasedHeapWithConvAs extends PointBasedHeap {

	/**
	 * Tracks the fields of each allocation site.
	 */
	private final Map<AllocationSite, Set<SymbolicExpression>> fields;

	/**
	 * Builds a new instance of field-sensitive point-based heap.
	 */
	public FieldSensitivePointBasedHeapWithConvAs() {
		super();
		this.fields = Collections.emptyMap();
	}

	/**
	 * Builds a new instance of field-sensitive point-based heap from its heap
	 * environment.
	 *
	 * @param heapEnv the heap environment that this instance tracks
	 */
	public FieldSensitivePointBasedHeapWithConvAs(
			HeapEnvironment<AllocationSites> heapEnv) {
		this(heapEnv, Collections.emptyMap());
	}

	/**
	 * Builds a new instance of field-sensitive point-based heap from its heap
	 * environment.
	 *
	 * @param heapEnv the heap environment that this instance tracks
	 * @param fields  the mapping between allocation sites and their fields that
	 *                    this instance tracks
	 */
	public FieldSensitivePointBasedHeapWithConvAs(
			HeapEnvironment<AllocationSites> heapEnv,
			Map<AllocationSite, Set<SymbolicExpression>> fields) {
		super(heapEnv);
		this.fields = fields;
	}

	/**
	 * Builds a new instance of field-sensitive point-based heap from its heap
	 * environment, its replacements, and its field mapping.
	 *
	 * @param heapEnv      the heap environment that this instance tracks
	 * @param replacements the heap replacements
	 * @param fields       the mapping between allocation sites and their fields
	 *                         that this instance tracks
	 */
	public FieldSensitivePointBasedHeapWithConvAs(
			HeapEnvironment<AllocationSites> heapEnv,
			List<HeapReplacement> replacements,
			Map<AllocationSite, Set<SymbolicExpression>> fields) {
		super(heapEnv, replacements);
		this.fields = fields;
	}

	@Override
	protected FieldSensitivePointBasedHeapWithConvAs buildHeapAfterAssignment(
			PointBasedHeap sss,
			List<HeapReplacement> replacements) {
		return new FieldSensitivePointBasedHeapWithConvAs(sss.heapEnv, replacements,
				((FieldSensitivePointBasedHeapWithConvAs) sss).fields);
	}

	@Override
	public FieldSensitivePointBasedHeapWithConvAs nonAliasedAssignment(
			Identifier id,
			StackAllocationSite site,
			PointBasedHeap pb,
			ProgramPoint pp,
			List<HeapReplacement> replacements)
			throws SemanticException {
		// no aliasing: star_y must be cloned and the clone must
		// be assigned to id
		StackAllocationSite clone = new StackAllocationSite(site.getStaticType(),
				id.getCodeLocation().toString(), site.isWeak(), id.getCodeLocation());
		HeapEnvironment<AllocationSites> heap = pb.heapEnv.assign(id, clone, pp);

		Map<AllocationSite,
				Set<SymbolicExpression>> newFields = new HashMap<>(
						((FieldSensitivePointBasedHeapWithConvAs) pb).fields);

		// all the allocation sites fields of star_y
		if (((FieldSensitivePointBasedHeapWithConvAs) pb).fields.containsKey(site)) {
			for (SymbolicExpression field : ((FieldSensitivePointBasedHeapWithConvAs) pb).fields.get(site)) {
				StackAllocationSite cloneWithField = new StackAllocationSite(field.getStaticType(),
						id.getCodeLocation().toString(), field, site.isWeak(), id.getCodeLocation());

				StackAllocationSite star_yWithField = new StackAllocationSite(field.getStaticType(),
						site.getCodeLocation().toString(), field, site.isWeak(),
						site.getCodeLocation());
				HeapReplacement replacement = new HeapReplacement();
				replacement.addSource(star_yWithField);
				replacement.addTarget(cloneWithField);
				replacement.addTarget(star_yWithField);

				// need to update also the fields of the clone
				addField(clone, field, newFields);

				replacements.add(replacement);
			}
		}

		// need to be replaced also the allocation site (needed for type
		// analysis)
		HeapReplacement replacement = new HeapReplacement();
		replacement.addSource(site);
		replacement.addTarget(clone);
		replacement.addTarget(site);
		replacements.add(replacement);

		return new FieldSensitivePointBasedHeapWithConvAs(heap, newFields);
	}

	@Override
	public FieldSensitivePointBasedHeapWithConvAs from(
			PointBasedHeap original) {
		return new FieldSensitivePointBasedHeapWithConvAs(original.heapEnv, fields);
	}

	@Override
	public ExpressionSet<ValueExpression> rewrite(
			SymbolicExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		return expression.accept(new Rewriter());
	}

	@Override
	public FieldSensitivePointBasedHeapWithConvAs smallStepSemantics(
			SymbolicExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		if (expression instanceof AccessChild) {
			FieldSensitivePointBasedHeapWithConvAs sss = (FieldSensitivePointBasedHeapWithConvAs) super.smallStepSemantics(
					expression, pp);

			AccessChild accessChild = (AccessChild) expression;
			Map<AllocationSite,
					Set<SymbolicExpression>> mapping = new HashMap<AllocationSite, Set<SymbolicExpression>>(sss.fields);

			ExpressionSet<ValueExpression> exprs = rewrite(accessChild.getContainer(), pp);
			for (ValueExpression rec : exprs) {
				if (rec instanceof BinaryExpression
						&& ((BinaryExpression) rec).getOperator().equals(TypeConv.INSTANCE)) {
					rec = (ValueExpression) ((BinaryExpression) rec).getLeft();
				}

				if (rec instanceof MemoryPointer) {
					AllocationSite site = (AllocationSite) ((MemoryPointer) rec).getReferencedLocation();
					ExpressionSet<ValueExpression> childs = rewrite(accessChild.getChild(), pp);

					for (ValueExpression child : childs)
						addField(site, child, mapping);

				} else if (rec instanceof AllocationSite) {
					AllocationSite site = (AllocationSite) rec;
					ExpressionSet<ValueExpression> childs = rewrite(accessChild.getChild(), pp);

					for (ValueExpression child : childs)
						addField(site, child, mapping);
				}
			}
			return new FieldSensitivePointBasedHeapWithConvAs(heapEnv, heapEnv.getSubstitution(), mapping);

		}

		PointBasedHeap sss = super.smallStepSemantics(expression, pp);
		return new FieldSensitivePointBasedHeapWithConvAs(sss.heapEnv, fields);
	}

	public class Rewriter extends PointBasedHeap.Rewriter {

		@Override
		public ExpressionSet<ValueExpression> visit(
				AccessChild expression,
				ExpressionSet<ValueExpression> receiver,
				ExpressionSet<ValueExpression> child,
				Object... params)
				throws SemanticException {
			Set<ValueExpression> result = new HashSet<>();

			for (ValueExpression rec : receiver) {
				if (rec instanceof BinaryExpression
						&& ((BinaryExpression) rec).getOperator().equals(TypeConv.INSTANCE)) {
					rec = (ValueExpression) ((BinaryExpression) rec).getLeft();
				}
				if (rec instanceof MemoryPointer) {
					AllocationSite site = (AllocationSite) ((MemoryPointer) rec).getReferencedLocation();
					populate(expression, child, result, site);
				} else if (rec instanceof AllocationSite) {
					AllocationSite site = (AllocationSite) rec;
					populate(expression, child, result, site);
				}
			}
			return new ExpressionSet<>(result);
		}

		private void populate(
				AccessChild expression,
				ExpressionSet<ValueExpression> child,
				Set<ValueExpression> result,
				AllocationSite site) {
			for (SymbolicExpression target : child) {
				AllocationSite e;

				if (site instanceof StackAllocationSite)
					e = new StackAllocationSite(
							expression.getStaticType(),
							site.getLocationName(),
							target,
							site.isWeak(),
							site.getCodeLocation());
				else
					e = new HeapAllocationSite(
							expression.getStaticType(),
							site.getLocationName(),
							target,
							site.isWeak(),
							site.getCodeLocation());

				if (expression.hasRuntimeTypes())
					e.setRuntimeTypes(expression.getRuntimeTypes(null));
				result.add(e);
			}
		}

		@Override
		public ExpressionSet<ValueExpression> visit(
				MemoryAllocation expression,
				Object... params)
				throws SemanticException {
			String pp = expression.getCodeLocation().getCodeLocation();

			boolean weak;
			if (alreadyAllocated(pp) != null)
				weak = true;
			else
				weak = false;

			AllocationSite e;
			if (expression.isStackAllocation())
				e = new StackAllocationSite(expression.getStaticType(), pp, weak, expression.getCodeLocation());
			else
				e = new HeapAllocationSite(expression.getStaticType(), pp, weak, expression.getCodeLocation());

			if (expression.hasRuntimeTypes())
				e.setRuntimeTypes(expression.getRuntimeTypes(null));
			return new ExpressionSet<>(e);
		}
	}

	@Override
	public FieldSensitivePointBasedHeapWithConvAs mk(
			PointBasedHeap reference) {
		if (reference instanceof FieldSensitivePointBasedHeapWithConvAs)
			return new FieldSensitivePointBasedHeapWithConvAs(reference.heapEnv,
					((FieldSensitivePointBasedHeapWithConvAs) reference).fields);
		else
			return new FieldSensitivePointBasedHeapWithConvAs(reference.heapEnv);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + Objects.hash(fields);
		return result;
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
		FieldSensitivePointBasedHeapWithConvAs other = (FieldSensitivePointBasedHeapWithConvAs) obj;
		return Objects.equals(fields, other.fields);
	}

	private void addField(
			AllocationSite site,
			SymbolicExpression field,
			Map<AllocationSite, Set<SymbolicExpression>> mapping) {
		if (!mapping.containsKey(site))
			mapping.put(site, new HashSet<>());
		mapping.get(site).add(field);
	}

	@Override
	public FieldSensitivePointBasedHeapWithConvAs popScope(
			ScopeToken scope)
			throws SemanticException {
		return new FieldSensitivePointBasedHeapWithConvAs(heapEnv.popScope(scope), fields);
	}

	@Override
	public FieldSensitivePointBasedHeapWithConvAs pushScope(
			ScopeToken scope)
			throws SemanticException {
		return new FieldSensitivePointBasedHeapWithConvAs(heapEnv.pushScope(scope), fields);
	}
}
