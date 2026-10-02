package it.unive.pylisa.cfg.statement;

import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.pylisa.cfg.statement.CallTargets.Resolution;
import it.unive.pylisa.cfg.statement.CallTargets.Target;
import it.unive.pylisa.cfg.statement.CallTargets.Unresolved;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Checks how the resolution of a method of a class becomes call targets,
 * including resolutions no small program produces.
 */
class MethodTargetsTest {

	@Test
	void methodFoundOnlyByItsNameHasAnUnresolvedPart() {
		List<Target> targets = CallTargets.methodTargets(
				new Resolution(null, Set.of(), "lib.C", "direct-registry"), "__init__", null);
		assertTrue(targets.stream().anyMatch(target -> target instanceof Unresolved unresolved
				&& unresolved.reason().contains("only by its name")), targets.toString());
	}

	@Test
	void methodNotFoundIsUnresolved() {
		List<Target> targets = CallTargets.methodTargets(
				new Resolution(null, Set.of(), "lib.C -> lib.A", "unresolved"), "__new__", null);
		assertTrue(targets.stream().anyMatch(target -> target instanceof Unresolved unresolved
				&& unresolved.reason().contains("no __new__")), targets.toString());
	}

	@Test
	void methodOfUnknownTypeIsUnresolved() {
		List<Target> targets = CallTargets.methodTargets(
				new Resolution(null, Set.of(NoInfoType.INSTANCE), "lib.C", "direct"), "__init__", null);
		assertTrue(targets.stream().allMatch(Unresolved.class::isInstance), targets.toString());
	}
}
