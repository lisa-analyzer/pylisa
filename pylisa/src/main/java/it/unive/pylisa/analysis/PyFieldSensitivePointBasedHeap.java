package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.heap.allocations.HeapAllocationSite;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;

/**
 * Python-specific {@link FieldSensitivePointBasedHeap} that is tolerant of
 * unresolved attribute accesses ({@link AccessChild}).
 * <p>
 * In dynamically-typed Python code it is very common to reach a call site like
 * {@code obj.attr(...)} where {@code obj} has no concrete allocation in the
 * heap lattice (for example because its value was an unresolved import, a
 * function parameter whose callers are unknown, or a global whose prior assign
 * did not produce a heap reference). The default rewriting rule for
 * {@link AccessChild} in {@link FieldSensitivePointBasedHeap} iterates over the
 * receiver's allocation sites and, when none are present, returns an empty
 * {@link ExpressionSet}.
 * <p>
 * Downstream,
 * {@link it.unive.lisa.analysis.SimpleAbstractDomain#smallStepSemantics
 * SimpleAbstractDomain.smallStepSemantics} interprets an empty rewrite as "this
 * expression has no meaningful rewriting" and returns {@code state.bottom()} —
 * i.e. the <em>full</em> abstract state (heap, value, type lattices) collapses
 * to bottom. For real Python programs this tends to cascade: dozens of
 * statements inside a module's {@code $init} CFG contain attribute accesses on
 * objects whose exact allocation site we cannot track, and every such access
 * bottoms the state. Once the value lattice is bottom, every subsequent write
 * in that CFG stays bottom, which in turn propagates to everything that
 * transitively imports the module.
 * <p>
 * <strong>Unsound fix:</strong> when the receiver of an {@code AccessChild}
 * cannot be resolved to any allocation, we synthesise a single weak
 * {@link HeapAllocationSite} keyed on the access's program point. A
 * {@code HeapAllocationSite} is a real {@link it.unive.lisa.symbolic.value.Identifier}
 * (it extends {@code HeapLocation}), so it satisfies the contract of
 * {@link it.unive.lisa.analysis.Analysis#assign} — which iterates the
 * rewrite result and rejects anything that is not an {@code Identifier}.
 * Pre-this change we returned a {@link PushAny} here, which is <em>not</em>
 * an {@code Identifier} and caused
 * {@code SemanticException("Rewriting … did not produce an identifier: PUSHANY")}
 * to be thrown for any assignment whose LHS rewrote through this fallback,
 * killing the whole analysis on real Python code (~8 repos in the bulk eval).
 * <p>
 * The synthetic site is marked weak so multiple writes into "the same
 * unresolved access" lub rather than overwrite, and it carries the access's
 * static type so the type domain stays consistent. The location name is
 * derived from the access's program point so repeated rewrites of the same
 * access produce the same identifier — necessary for the value-domain to
 * recognise re-reads. This is deliberately unsound: we lose the precision
 * of "obj.attr must alias these specific sites", but we preserve the
 * reachability of the analysis state and the information carried by value,
 * type, and heap lattices for the <em>rest</em> of the CFG. Without this
 * fallback we observed ~18k bot-state transitions that cascaded into 140+
 * submodule {@code $init}s inheriting a bottom value lattice.
 * <p>
 * All other rewriting cases are delegated unchanged to the parent class.
 */
public class PyFieldSensitivePointBasedHeap extends FieldSensitivePointBasedHeap {

	private static final java.util.concurrent.atomic.AtomicInteger HITS = new java.util.concurrent.atomic.AtomicInteger();
	// Diagnostic: count empty-rewrite cases NOT covered by the AccessChild
	// fallback below, bucketed by expression class. Each unique class is
	// logged on its first occurrence and every 500th occurrence; this
	// surfaces which symbolic-expression families land in lisa-sdk's
	// `SimpleAbstractDomain.smallStepSemantics` empty-rewrite → ⊥ path
	// (the documented cascade source — see the class-level javadoc). On
	// IBM/mcp-context-forge specifically, the AccessChild fallback alone
	// is not enough to keep the state non-⊥ through Settings(BaseSettings)
	// instantiation, so we need to learn what else is here.
	private static final java.util.concurrent.ConcurrentHashMap<String, java.util.concurrent.atomic.AtomicInteger> UNCOVERED_HITS =
			new java.util.concurrent.ConcurrentHashMap<>();

	@Override
	public ExpressionSet rewrite(
			HeapEnvWithFields state,
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ExpressionSet result = super.rewrite(state, expression, pp, oracle);
		if (result.isEmpty() && expression instanceof AccessChild) {
			int n = HITS.incrementAndGet();
			if (n == 1 || n % 500 == 0)
				org.apache.logging.log4j.LogManager.getLogger(PyFieldSensitivePointBasedHeap.class).info(
						"[PYHEAP-FALLBACK] hits={} expr={} pp={}", n, expression, pp.getLocation());
			// We don't know which allocation site the receiver points to.
			// Materialise a fresh weak HeapAllocationSite keyed on this
			// program point so the result is a valid Identifier — PushAny
			// would not be, and lisa-sdk's Analysis.assign throws on
			// non-Identifier rewrites (which used to kill ~8 repos in the
			// bulk eval). The site's name is stable across re-rewrites of
			// the same access (same pp → same name) so the value-domain
			// can recognise re-reads. See the class-level javadoc for the
			// full rationale and soundness trade-off.
			String name = "$pyheap@" + pp.getLocation();
			return new ExpressionSet(new HeapAllocationSite(
					expression.getStaticType(),
					name,
					true,
					expression.getCodeLocation()));
		}
		// Diagnostic-only: report empty rewrites for OTHER expression types,
		// which still cascade to ⊥ via lisa-sdk's empty-rewrite path. No
		// semantic effect — `result` is returned as-is — but the log
		// surfaces the expression families we need to cover next.
		if (result.isEmpty()) {
			String cls = expression == null ? "(null)" : expression.getClass().getSimpleName();
			java.util.concurrent.atomic.AtomicInteger ctr =
					UNCOVERED_HITS.computeIfAbsent(cls, k -> new java.util.concurrent.atomic.AtomicInteger());
			int n = ctr.incrementAndGet();
			if (n == 1 || n % 500 == 0)
				org.apache.logging.log4j.LogManager.getLogger(PyFieldSensitivePointBasedHeap.class).info(
						"[PYHEAP-UNCOVERED] class={} hits={} expr={} pp={}",
						cls, n, expression, pp.getLocation());
		}
		return result;
	}
}
