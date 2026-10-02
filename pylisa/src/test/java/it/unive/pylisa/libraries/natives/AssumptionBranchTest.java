package it.unive.pylisa.libraries.natives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Checks the statement that stands for a call on a branch depending on
 * assumptions: it keeps apart what the call raises on different branches and
 * what different calls raise, and never changes the call.
 */
class AssumptionBranchTest {

	private final CFG cfg = new CFG(new CodeMemberDescriptor(PySyntheticLocation.INSTANCE,
			new Program(new PythonFeatures(), new PythonTypeSystem()), false, "f"));

	private final SourceCodeLocation here = new SourceCodeLocation("f.py", 3, 4);

	private final VariableRef call = new VariableRef(cfg, here, "x");

	@Test
	void sameCallAndAssumptionsAreOneStatement() {
		assertEquals(new AssumptionBranch(call, Set.of("a", "b")), new AssumptionBranch(call, Set.of("b", "a")));
		assertEquals(new AssumptionBranch(call, Set.of("a")).hashCode(),
				new AssumptionBranch(call, Set.of("a")).hashCode());
	}

	@Test
	void differentAssumptionsAreDifferentStatements() {
		AssumptionBranch a = new AssumptionBranch(call, Set.of("a"));
		AssumptionBranch b = new AssumptionBranch(call, Set.of("b"));
		assertNotEquals(a, b);
		assertEquals(-Integer.signum(a.compareTo(b)), Integer.signum(b.compareTo(a)));
		assertNotEquals(0, a.compareTo(b));
	}

	@Test
	void orderAgreesWithEqualityWhateverTheNames() {
		AssumptionBranch joined = new AssumptionBranch(call, Set.of("a,b"));
		AssumptionBranch split = new AssumptionBranch(call, Set.of("a", "b"));
		assertNotEquals(joined, split);
		assertNotEquals(0, joined.compareTo(split));
		assertEquals(-Integer.signum(joined.compareTo(split)), Integer.signum(split.compareTo(joined)));
		AssumptionBranch prefix = new AssumptionBranch(call, Set.of("a"));
		assertNotEquals(0, prefix.compareTo(split));
		assertEquals(0, split.compareTo(new AssumptionBranch(call, Set.of("b", "a"))));
	}

	@Test
	void callsThatDifferAreDifferentStatementsAtOneLocation() {
		VariableRef other = new VariableRef(cfg, here, "y");
		assertNotEquals(new AssumptionBranch(call, Set.of("a")), new AssumptionBranch(other, Set.of("a")));
	}

	@Test
	void theCallIsItsParentAndIsNotChanged() {
		AssumptionBranch branch = new AssumptionBranch(call, Set.of("a"));
		assertSame(call, branch.getParentStatement());
		assertEquals(null, call.getParentStatement());
		assertSame(cfg, branch.getCFG());
		assertEquals(here, branch.getLocation());
	}
}
