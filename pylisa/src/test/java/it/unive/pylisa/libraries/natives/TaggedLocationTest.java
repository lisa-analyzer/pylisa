package it.unive.pylisa.libraries.natives;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.SourceCodeLocation;
import it.unive.pylisa.program.PySyntheticLocation;
import org.junit.jupiter.api.Test;

/**
 * Checks that a tagged location is the source position of its base, named with its tag, and that
 * it equals only a tagged location with the same base and tag.
 */
class TaggedLocationTest {

	private static final SourceCodeLocation BASE = new SourceCodeLocation("prog.py", 5, 19);

	@Test
	void aTaggedLocationIsThePositionOfItsBase() {
		TaggedLocation tagged = new TaggedLocation(BASE, "helper");
		assertEquals("prog.py", tagged.getSourceFile());
		assertEquals(5, tagged.getLine());
		assertEquals(19, tagged.getCol());
		assertEquals(BASE.getCodeLocation() + "#helper", tagged.getCodeLocation());
	}

	@Test
	void equalityNeedsTheSameBaseAndTag() {
		TaggedLocation tagged = new TaggedLocation(BASE, "helper");
		assertEquals(tagged, new TaggedLocation(new SourceCodeLocation("prog.py", 5, 19), "helper"));
		assertNotEquals(tagged, new TaggedLocation(BASE, "other"));
		assertNotEquals(tagged, BASE);
		assertNotEquals(BASE, tagged);
	}

	@Test
	void aBaseThatIsNoSourcePositionHasNoLine() {
		TaggedLocation tagged = new TaggedLocation(PySyntheticLocation.INSTANCE, "x");
		assertEquals(-1, tagged.getLine());
		assertTrue(tagged.getCodeLocation().endsWith("#x"));
	}

	@Test
	void tagsOrderLocationsAtTheSamePosition() {
		TaggedLocation a = new TaggedLocation(BASE, "a");
		TaggedLocation b = new TaggedLocation(BASE, "b");
		assertTrue(a.compareTo(b) < 0 && b.compareTo(a) > 0);
		assertEquals(0, a.compareTo(new TaggedLocation(BASE, "a")));
		assertTrue(a.compareTo(BASE) > 0, "a tagged location follows its plain position");
	}
}
