package it.unive.pylisa.analysis.types;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Tests that the order of {@link PythonTypeSet} reads a set holding {@link NoInfoType} as every
 * type, as the environments do when they read an identifier they do not hold.
 */
class PythonTypeSetTest {

	private static final PythonTypeSet BOOL = set(BoolType.INSTANCE);

	private static final PythonTypeSet STRING = set(StringType.INSTANCE);

	private static final PythonTypeSet NO_INFO_AND_BOOL = set(NoInfoType.INSTANCE, BoolType.INSTANCE);

	@Test
	void everySetIsBelowTheUnknownTypes() throws SemanticException {
		assertTrue(BOOL.lessOrEqual(PythonTypeSet.NO_INFO_TYPE));
		assertTrue(BOOL.lessOrEqual(set(NoInfoType.INSTANCE, StringType.INSTANCE)));
		assertTrue(NO_INFO_AND_BOOL.lessOrEqual(PythonTypeSet.NO_INFO_TYPE));
		assertTrue(PythonTypeSet.NO_INFO_TYPE.lessOrEqual(NO_INFO_AND_BOOL));
	}

	@Test
	void theUnknownTypesAreNotBelowAKnownSet() throws SemanticException {
		assertFalse(PythonTypeSet.NO_INFO_TYPE.lessOrEqual(BOOL));
		assertFalse(NO_INFO_AND_BOOL.lessOrEqual(BOOL));
	}

	@Test
	void theLeastUpperBoundIsAboveBothSets() throws SemanticException {
		for (PythonTypeSet a : Set.of(BOOL, STRING, PythonTypeSet.NO_INFO_TYPE, NO_INFO_AND_BOOL))
			for (PythonTypeSet b : Set.of(BOOL, STRING, PythonTypeSet.NO_INFO_TYPE, NO_INFO_AND_BOOL)) {
				PythonTypeSet lub = a.lub(b);
				assertTrue(a.lessOrEqual(lub), a + " <= " + a + " lub " + b);
				assertTrue(b.lessOrEqual(lub), b + " <= " + a + " lub " + b);
			}
	}

	@Test
	void anIdentifierMissingFromAnEnvironmentIsReadAsAboveEverySet() throws SemanticException {
		assertTrue(BOOL.lessOrEqual(BOOL.unknownValue(null)));
	}

	@Test
	void theGreatestLowerBoundWithTheUnknownTypesIsTheOtherSet() throws SemanticException {
		assertEquals(BOOL, PythonTypeSet.NO_INFO_TYPE.glb(BOOL));
		assertEquals(BOOL, BOOL.glb(NO_INFO_AND_BOOL));
	}

	private static PythonTypeSet set(
			Type... types) {
		return new PythonTypeSet(false, Set.of(types));
	}
}
