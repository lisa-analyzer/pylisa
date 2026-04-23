package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.pointbased.FieldSensitivePointBasedHeap;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.heap.allocations.HeapEnvWithFields;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.value.PushAny;

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
 * cannot be resolved to any allocation, we return a single {@link PushAny}
 * expression with the same static type as the access. {@code PushAny} is the
 * standard "we don't know" placeholder: value domains evaluate it to top, type
 * domains keep the declared type, and the heap domain treats it as opaque. This
 * is deliberately unsound — we lose the precision of "obj.attr must alias these
 * specific sites" — but it preserves the reachability of the analysis state and
 * therefore the information carried by value, type, and heap lattices for the
 * <em>rest</em> of the CFG. Without this fallback, we observed ~18k bot-state
 * transitions that cascaded into 140+ submodule {@code $init}s inheriting a
 * bottom value lattice.
 * <p>
 * All other rewriting cases are delegated unchanged to the parent class.
 */
public class PyFieldSensitivePointBasedHeap extends FieldSensitivePointBasedHeap {

	private static final java.util.concurrent.atomic.AtomicInteger HITS = new java.util.concurrent.atomic.AtomicInteger();

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
			// unsound: we do not know which allocation site the receiver
			// refers to; returning the expression itself is rejected by the
			// single-rewrite branch (AccessChild is a HeapExpression, not a
			// ValueExpression), so we fall back to PushAny to keep the
			// analysis reachable. See the class-level javadoc for the full
			// rationale.
			return new ExpressionSet(new PushAny(expression.getStaticType(), expression.getCodeLocation()));
		}
		return result;
	}
}
