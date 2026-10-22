package it.unive.pylisa.analysis.namespaces;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import it.unive.lisa.analysis.SemanticException;
import org.junit.Test;

public class FrameTest {

	@Test
	public void emptyFrameHasNoBoundName() {
		assertEquals(Binding.UNBOUND, Frame.EMPTY.status("x"));
		assertFalse(Frame.EMPTY.isTop());
		assertFalse(Frame.EMPTY.isBottom());
		assertNotEquals(Frame.EMPTY, Frame.BOTTOM);
	}

	@Test
	public void topAndBottomStatuses() {
		assertEquals(Binding.MAYBE, Frame.TOP.status("x"));
		assertEquals(Binding.BOTTOM, Frame.BOTTOM.status("x"));
		assertSame(Frame.BOTTOM, Frame.BOTTOM.bind("x"));
		assertSame(Frame.BOTTOM, Frame.BOTTOM.havoc());
	}

	@Test
	public void bindAndUnbind() {
		Frame f = Frame.EMPTY.bind("x");
		assertEquals(Binding.BOUND, f.status("x"));
		assertEquals(Binding.UNBOUND, f.status("y"));
		assertEquals(Binding.UNBOUND, f.unbind("x").status("x"));
		assertEquals(Frame.EMPTY, f.unbind("x"));
		assertSame(f, f.bind("x"));
	}

	@Test
	public void lubOfBoundAndUnboundIsMaybe()
			throws SemanticException {
		Frame x = Frame.EMPTY.bind("x");
		Frame y = Frame.EMPTY.bind("y");
		Frame lub = x.lub(y);
		assertEquals(Binding.MAYBE, lub.status("x"));
		assertEquals(Binding.MAYBE, lub.status("y"));
		assertEquals(Binding.UNBOUND, lub.status("z"));
		// regression: with "missing = unbound", the lub with the empty frame
		// must not be the other operand
		assertEquals(Binding.MAYBE, Frame.EMPTY.lub(x).status("x"));
		assertEquals(Binding.BOUND, x.lub(x.bind("x")).status("x"));
	}

	@Test
	public void lessOrEqualFollowsTheStatuses()
			throws SemanticException {
		Frame x = Frame.EMPTY.bind("x");
		Frame maybeX = x.lub(Frame.EMPTY);
		assertTrue(x.lessOrEqual(maybeX));
		assertTrue(Frame.EMPTY.lessOrEqual(maybeX));
		assertFalse(maybeX.lessOrEqual(x));
		// bound and unbound are not comparable
		assertFalse(x.lessOrEqual(Frame.EMPTY));
		assertFalse(Frame.EMPTY.lessOrEqual(x));
		assertTrue(x.lessOrEqual(Frame.TOP));
		assertTrue(Frame.BOTTOM.lessOrEqual(Frame.EMPTY));
	}

	@Test
	public void havocMakesEveryNameMaybeBound()
			throws SemanticException {
		Frame x = Frame.EMPTY.bind("x");
		Frame h = x.havoc();
		assertEquals(Binding.MAYBE, h.status("x"));
		assertEquals(Binding.MAYBE, h.status("anything"));
		assertTrue(h.isHavocked());
		// binding after the havoc
		Frame hy = h.bind("y");
		assertEquals(Binding.BOUND, hy.status("y"));
		assertEquals(Binding.MAYBE, hy.status("z"));
		assertTrue(hy.isHavocked());
		// unbinding under havoc is sound, but imprecise
		assertEquals(Binding.MAYBE, hy.unbind("y").status("y"));
		// the lub with a havocked frame is havocked
		assertTrue(x.lub(hy).isHavocked());
		assertEquals(Binding.MAYBE, x.lub(hy).status("x"));
		assertTrue(x.lessOrEqual(h));
		assertFalse(h.lessOrEqual(x));
		assertTrue(hy.lessOrEqual(h));
		assertFalse(h.lessOrEqual(hy));
	}

	@Test
	public void glbOfIncompatibleFramesIsBottom()
			throws SemanticException {
		Frame x = Frame.EMPTY.bind("x");
		assertTrue(x.glb(Frame.EMPTY).isBottom());
		Frame maybeX = x.lub(Frame.EMPTY);
		assertEquals(x, maybeX.glb(x));
		assertEquals(Frame.EMPTY, maybeX.glb(Frame.EMPTY));
		assertEquals(x, Frame.TOP.glb(x));
	}
}
