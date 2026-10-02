package it.unive.pylisa.cfg.statement;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import it.unive.pylisa.cfg.type.PyFunctionType;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PyClassUnit;
import it.unive.pylisa.program.PySyntheticLocation;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

/**
 * Checks that the statements pylisa builds for code with no place in a source
 * file stay distinct statements: they all share one synthetic location, so the
 * methods of one class body, or the classes of one library module, would
 * otherwise be one statement for the per-statement results of the analysis,
 * and each would read the state of another.
 */
class SyntheticStatementIdentityTest {

	private static final PySyntheticLocation HERE = PySyntheticLocation.INSTANCE;

	private final Program program = new Program(new PythonFeatures(), new PythonTypeSystem());

	private final CFG cfg = new CFG(new CodeMemberDescriptor(HERE, program, false, "$init"));

	@AfterEach
	void forgetFunctionTypes() {
		PyFunctionType.clearAll();
	}

	@Test
	void methodsOfOneClassBodyAreDistinctStatements() {
		FunctionUnit init = function("C.__init__");
		assertDistinct(new ImportFunction(cfg, HERE, "C.__init__", init),
				new ImportFunction(cfg, HERE, "C.cb", function("C.cb")));
		assertEquals(new ImportFunction(cfg, HERE, "C.__init__", init),
				new ImportFunction(cfg, HERE, "C.__init__", init));
	}

	@Test
	void methodsOfOneLibraryClassBodyAreDistinctStatements() {
		// the library loader names these statements after the class, not the
		// method
		assertDistinct(new ImportFunction(cfg, HERE, "lib.C", function("lib.C.open")),
				new ImportFunction(cfg, HERE, "lib.C", function("lib.C.close")));
	}

	@Test
	void classesOfOneModuleAreDistinctStatements() {
		ImportClass first = new ImportClass(cfg, HERE, "A", new PyClassUnit(HERE, program, "m.A", false));
		ImportClass second = new ImportClass(cfg, HERE, "B", new PyClassUnit(HERE, program, "m.B", false));
		assertDistinct(first, second);
	}

	@Test
	void importsOfTwoModulesAreDistinctStatements() {
		ImportModule first = new ImportModule(cfg, HERE, "a", new ModuleUnit(HERE, program, "a"));
		ImportModule second = new ImportModule(cfg, HERE, "b", new ModuleUnit(HERE, program, "b"));
		assertDistinct(first, second);
	}

	@Test
	void literalsOfTwoFunctionsAreOrderedBothWays() {
		FunctionLiteral first = new FunctionLiteral(cfg, HERE, registered("m.f"));
		FunctionLiteral second = new FunctionLiteral(cfg, HERE, registered("m.g"));
		assertDistinct(first, second);
	}

	@Test
	void theSyntheticLocationPrintsAndHashesTheSameInEveryRun() {
		assertEquals("<synthetic>", HERE.toString());
		assertEquals("<synthetic>".hashCode(), HERE.hashCode());
	}

	private FunctionUnit function(
			String name) {
		return new FunctionUnit(HERE, program, name, false);
	}

	private FunctionUnit registered(
			String name) {
		FunctionUnit unit = function(name);
		PyFunctionType.register(name, unit);
		return unit;
	}

	private static void assertDistinct(
			Statement first,
			Statement second) {
		assertNotEquals(first, second);
		int forward = first.compareTo(second);
		assertNotEquals(0, forward);
		assertTrue(Integer.signum(forward) == -Integer.signum(second.compareTo(first)),
				"the order of " + first + " and " + second + " is not antisymmetric");
	}
}
