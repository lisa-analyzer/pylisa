package it.unive.pylisa;

import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.AllocationSites;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.analysis.heap.pointbased.HeapAllocationSite;
import it.unive.lisa.analysis.heap.pointbased.StackAllocationSite;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.lattices.GenericMapLattice;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
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
import java.util.Set;

public class PyFieldSensitivePointBasedHeap extends FieldSensitivePointBasedHeap {

	public PyFieldSensitivePointBasedHeap() {
		super();
	}

	private PyFieldSensitivePointBasedHeap(
			HeapEnvironment<AllocationSites> heapEnv,
			List<HeapReplacement> replacements,
			GenericMapLattice<AllocationSite, ExpressionSet> fields) {
		super(heapEnv, replacements, fields);
	}

/*@Override
	public PyFieldSensitivePointBasedHeap popScope(
			ScopeToken scope)
			throws SemanticException {
		return this;
	}

	@Override
	public PyFieldSensitivePointBasedHeap pushScope(
			ScopeToken scope)
			throws SemanticException {
		return this;
	}*/

	@Override
	public FieldSensitivePointBasedHeap mk(
			FieldSensitivePointBasedHeap reference) {
		return new PyFieldSensitivePointBasedHeap(reference.heapEnv, Collections.emptyList(), reference.fields);
	}

	@Override
	public FieldSensitivePointBasedHeap mk(
			FieldSensitivePointBasedHeap reference,
			List<HeapReplacement> replacements) {
		return new PyFieldSensitivePointBasedHeap(reference.heapEnv, replacements, reference.fields);
	}

	@Override
	protected FieldSensitivePointBasedHeap mk(
			FieldSensitivePointBasedHeap reference,
			HeapEnvironment<AllocationSites> heapEnv) {
		return new PyFieldSensitivePointBasedHeap(heapEnv, Collections.emptyList(), reference.fields);
	}

	// the following is to support TypeConv on `this`

	private void addField(
			AllocationSite site,
			SymbolicExpression field,
			Map<AllocationSite, ExpressionSet> mapping) {
		Set<SymbolicExpression> tmp = new HashSet<>(mapping.getOrDefault(site, new ExpressionSet()).elements());
		tmp.add(field);
		mapping.put(site, new ExpressionSet(tmp));
	}

	@Override
	public FieldSensitivePointBasedHeap smallStepSemantics(
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (expression instanceof AccessChild) {
			PyFieldSensitivePointBasedHeap sss = (PyFieldSensitivePointBasedHeap) super.smallStepSemantics(
					expression,
					pp,
					oracle);

			AccessChild accessChild = (AccessChild) expression;
			Map<AllocationSite, ExpressionSet> mapping = new HashMap<>(sss.fields.getMap());

			ExpressionSet exprs = rewrite(accessChild.getContainer(), pp, oracle);
			for (SymbolicExpression rec : exprs) {
				if (rec instanceof BinaryExpression
						&& ((BinaryExpression) rec).getOperator().equals(TypeConv.INSTANCE)) {
					// get rid of the conversion
					rec = (ValueExpression) ((BinaryExpression) rec).getLeft();
				}

				if (rec instanceof MemoryPointer) {
					AllocationSite site = (AllocationSite) ((MemoryPointer) rec).getReferencedLocation();
					ExpressionSet childs = rewrite(accessChild.getChild(), pp, oracle);

					for (SymbolicExpression child : childs)
						addField(site, child, mapping);
				} else if (rec instanceof AllocationSite) {
					AllocationSite site = (AllocationSite) rec;
					ExpressionSet childs = rewrite(accessChild.getChild(), pp, oracle);

					for (SymbolicExpression child : childs)
						addField(site, child, mapping);
				}
			}
			return new PyFieldSensitivePointBasedHeap(
					heapEnv,
					heapEnv.getSubstitution(),
					new GenericMapLattice<>(fields.lattice, mapping));
		}

		FieldSensitivePointBasedHeap sss = super.smallStepSemantics(expression, pp, oracle);
		return new PyFieldSensitivePointBasedHeap(sss.heapEnv, sss.replacements, sss.fields);
	}

	@Override
	public ExpressionSet rewrite(
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return expression.accept(new Rewriter());
	}

	public class Rewriter extends FieldSensitivePointBasedHeap.Rewriter {
		@Override
		public ExpressionSet visit(
				AccessChild expression,
				ExpressionSet receiver,
				ExpressionSet child,
				Object... params)
				throws SemanticException {
			Set<SymbolicExpression> result = new HashSet<>();

			for (SymbolicExpression rec : receiver) {
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
			return new ExpressionSet(result);
		}

		private void populate(
				AccessChild expression,
				ExpressionSet child,
				Set<SymbolicExpression> result,
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

				// propagates the annotations of the child value expression to
				// the newly created allocation site
				if (target instanceof Identifier)
					for (Annotation ann : e.getAnnotations())
						e.addAnnotation(ann);

				result.add(e);
			}
		}
	}
}