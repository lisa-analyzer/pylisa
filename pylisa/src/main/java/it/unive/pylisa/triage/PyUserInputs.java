package it.unive.pylisa.triage;

import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.triage.inputs.UserInputs;
import it.unive.pylisa.libraries.Input;
import it.unive.pylisa.libraries.svverifier.NondetBool;
import it.unive.pylisa.libraries.svverifier.NondetFloat;
import it.unive.pylisa.libraries.svverifier.NondetFloatRange;
import it.unive.pylisa.libraries.svverifier.NondetInt;
import it.unive.pylisa.libraries.svverifier.NondetList;
import it.unive.pylisa.libraries.svverifier.NondetStr;
import java.util.Set;

/**
 * The natives of the Python libraries whose unknown results are inputs provided
 * by the user (see {@link UserInputs}): {@code input()}, that reads the
 * standard input, and the nondeterministic choices of {@code _sv_verifier}.
 *
 * @author <a href="mailto:luca.negrini@unive.it">Luca Negrini</a>
 */
public class PyUserInputs
		implements
		UserInputs {

	/**
	 * The constructs of the natives producing user inputs.
	 */
	private static final Set<Class<?>> INPUTS = Set.of(Input.class, NondetBool.class, NondetFloat.class,
			NondetFloatRange.class, NondetInt.class, NondetList.class, NondetStr.class);

	@Override
	public boolean isUserInput(
			NativeCFG target) {
		return INPUTS.contains(target.getConstruct());
	}
}
