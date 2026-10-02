package it.unive.pylisa.symbolic;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.SourceCodeLocation;
import org.junit.jupiter.api.Test;

/**
 * Checks that the value of {@code None} is told apart from any other value.
 */
class PyNoneConstantTest {

	@Test
	void theValueOfNoneIsRecognised() {
		assertTrue(PyNoneConstant.isNoneValue(new PyNoneConstant(new SourceCodeLocation("p.py", 1, 0)).getValue()));
		assertFalse(PyNoneConstant.isNoneValue(new Object()));
		assertFalse(PyNoneConstant.isNoneValue(null));
	}
}
