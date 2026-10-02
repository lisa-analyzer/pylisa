package it.unive.pylisa.program;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.pylisa.frontend.PyFrontend;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Checks the settings a translated program carries for library models.
 */
class ProgramSettingsTest {

	@Test
	void aSettingIsFoundByTheTypeItWasRegisteredUnder() {
		ProgramSettings settings = ProgramSettings.NONE.with(CharSequence.class, "value");
		assertEquals(Optional.of("value"), settings.get(CharSequence.class));
		assertEquals(Optional.empty(), settings.get(String.class));
	}

	@Test
	void twoSettingsOfOneTypeAreRejected() {
		ProgramSettings settings = ProgramSettings.NONE.with(String.class, "first");
		assertThrows(IllegalArgumentException.class, () -> settings.with(String.class, "second"));
	}

	@Test
	void addingASettingLeavesTheOriginalUnchanged() {
		ProgramSettings.NONE.with(String.class, "value");
		assertTrue(ProgramSettings.NONE.get(String.class).isEmpty());
	}

	@Test
	void theTranslatedProgramCarriesTheSettings() throws Exception {
		ProgramSettings settings = ProgramSettings.NONE.with(String.class, "value");
		Program program = new PyFrontend("src/test/resources/programs/python/implicit_none_return.py", settings)
				.toLiSAProgram();
		assertSame(settings, assertInstanceOf(PyProgram.class, program).settings());
	}
}
