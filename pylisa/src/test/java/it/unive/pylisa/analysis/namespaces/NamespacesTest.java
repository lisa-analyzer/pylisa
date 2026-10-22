package it.unive.pylisa.analysis.namespaces;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import it.unive.lisa.analysis.FixpointInfo;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import java.util.Map;
import org.junit.Test;

public class NamespacesTest {

	// the program point is not used by the scoping operations
	private static final ProgramPoint PP = null;

	private static final ScopeToken CALL1 = new ScopeToken(() -> SyntheticLocation.INSTANCE);

	private static final ScopeToken CALL2 = new ScopeToken(() -> SyntheticLocation.INSTANCE);

	@Test
	public void missingModulesAreEmpty()
			throws SemanticException {
		assertEquals(Frame.EMPTY, Namespaces.EMPTY.getModule("m"));
		assertEquals(Binding.UNBOUND, Namespaces.EMPTY.status("m", "x", true));
		Namespaces m = Namespaces.EMPTY.bind("m", "x", true);
		assertEquals(Binding.BOUND, m.status("m", "x", true));
		assertEquals(Binding.UNBOUND, m.status("n", "x", true));
		assertEquals(Binding.UNBOUND, m.status("m", "x", false));
		// the lub with a module missing on one side is the lub with the
		// empty frame
		assertEquals(Binding.MAYBE, Namespaces.EMPTY.lub(m).status("m", "x", true));
		assertEquals(Binding.MAYBE, m.lub(Namespaces.EMPTY).status("m", "x", true));
		// a module whose frame is empty is not stored
		assertEquals(Namespaces.EMPTY, m.unbind("m", "x", true));
	}

	@Test
	public void localsAreSeparatedFromGlobals() {
		Namespaces n = Namespaces.EMPTY.bind("m", "x", false);
		assertEquals(Binding.BOUND, n.status("m", "x", false));
		assertEquals(Binding.UNBOUND, n.status("m", "x", true));
		assertEquals(Binding.BOUND, n.getLocals().status("x"));
	}

	@Test
	public void callsSaveAndRestoreTheLocals()
			throws SemanticException {
		Namespaces caller = Namespaces.EMPTY.bind("m", "x", false).bind("m", "g", true);
		Namespaces callee = caller.pushScope(CALL1, PP);
		// the callee starts with no local
		assertEquals(Frame.EMPTY, callee.getLocals());
		// the globals are shared
		assertEquals(Binding.BOUND, callee.status("m", "g", true));
		// the callee binds a local and a global
		Namespaces exit = callee.bind("m", "y", false).bind("m", "h", true);
		Namespaces back = exit.popScope(CALL1, PP);
		assertEquals(Binding.BOUND, back.status("m", "x", false));
		assertEquals(Binding.UNBOUND, back.status("m", "y", false));
		assertEquals(Binding.BOUND, back.status("m", "h", true));
		assertEquals(caller.bind("m", "h", true), back);
	}

	@Test
	public void pushingAnUsedTokenMergesTheSavedFrames()
			throws SemanticException {
		// e.g. a recursive call through the same call site
		Namespaces outer = Namespaces.EMPTY.bind("m", "x", false).pushScope(CALL1, PP);
		Namespaces inner = outer.bind("m", "y", false).pushScope(CALL1, PP);
		Namespaces back = inner.popScope(CALL1, PP);
		assertEquals(Binding.MAYBE, back.status("m", "x", false));
		assertEquals(Binding.MAYBE, back.status("m", "y", false));
	}

	@Test
	public void poppingAMissingTokenIsSound()
			throws SemanticException {
		Namespaces back = Namespaces.EMPTY.popScope(CALL1, PP);
		assertEquals(Binding.MAYBE, back.status("m", "anything", false));
	}

	@Test
	public void lessOrEqualConsidersTheSavedFrames()
			throws SemanticException {
		Namespaces one = Namespaces.EMPTY.pushScope(CALL1, PP);
		Namespaces two = Namespaces.EMPTY.pushScope(CALL2, PP);
		assertFalse(one.lessOrEqual(two));
		Namespaces lub = one.lub(two);
		assertTrue(one.lessOrEqual(lub));
		assertTrue(two.lessOrEqual(lub));
		assertEquals(Frame.EMPTY, lub.popScope(CALL1, PP).getLocals());
		assertTrue(Namespaces.EMPTY.lessOrEqual(Namespaces.TOP));
		assertTrue(Namespaces.BOTTOM.lessOrEqual(Namespaces.EMPTY));
	}

	@Test
	public void havocAffectsOnlyTheGivenModule() {
		Namespaces n = Namespaces.EMPTY.bind("m", "x", true).bind("n", "x", true).havocModule("m");
		assertEquals(Binding.MAYBE, n.status("m", "x", true));
		assertEquals(Binding.MAYBE, n.status("m", "y", true));
		assertEquals(Binding.BOUND, n.status("n", "x", true));
		assertEquals(Binding.UNBOUND, n.status("n", "y", true));
	}

	@Test
	public void topAndBottom() {
		assertEquals(Binding.MAYBE, Namespaces.TOP.status("m", "x", true));
		assertEquals(Binding.MAYBE, Namespaces.TOP.status("m", "x", false));
		assertEquals(Binding.BOTTOM, Namespaces.BOTTOM.status("m", "x", true));
		assertSame(Namespaces.TOP, Namespaces.TOP.bind("m", "x", true));
		assertSame(Namespaces.BOTTOM, Namespaces.BOTTOM.bind("m", "x", false));
	}

	@Test
	public void fixpointInfoForwardsTheScopingOperations()
			throws SemanticException {
		Namespaces caller = Namespaces.EMPTY.bind("m", "x", false);
		FixpointInfo info = new FixpointInfo(Map.of(Namespaces.INFO_KEY, caller));
		FixpointInfo pushed = info.pushScope(CALL1, PP);
		Namespaces callee = pushed.get(Namespaces.INFO_KEY, Namespaces.class);
		assertEquals(Frame.EMPTY, callee.getLocals());
		FixpointInfo popped = pushed.popScope(CALL1, PP);
		assertEquals(caller, popped.get(Namespaces.INFO_KEY, Namespaces.class));
	}
}
