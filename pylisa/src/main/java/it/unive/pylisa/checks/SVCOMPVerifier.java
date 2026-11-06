package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.checks.verification.ErrorViolation;
import it.unive.lisa.checks.verification.VerificationResult;
import it.unive.lisa.checks.verification.WarningVerifier;
import it.unive.lisa.outputs.messages.StatementMessage;
import it.unive.lisa.triage.FirstViolationVerifier;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The {@link WarningVerifier} used for SV-COMP tasks, as the one of JLiSA. Each
 * task has a single property (assertion safety, dynamic typing, arithmetic or
 * data-structure lookups, see {@link ExceptionChecker}), and the warnings about
 * the exceptions of each property (that is, of each category of
 * {@link ExceptionChecker#category(String)}) are verified separately, each with
 * a {@link FirstViolationVerifier}: the verification of the warnings of each
 * property stops at the first one that is confirmed, independently of the other
 * properties.
 *
 * @param <A> the kind of {@link AbstractLattice} produced by the domain
 *                {@code D}
 * @param <D> the kind of {@link AbstractDomain} ran during the analysis
 */
public class SVCOMPVerifier<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		WarningVerifier<A, D> {

	private final WarningVerifier<A, D> verifier;

	/**
	 * Builds the verifier, verifying the warnings of each property with a
	 * {@link FirstViolationVerifier} with default parameters.
	 */
	public SVCOMPVerifier() {
		this(new FirstViolationVerifier<>());
	}

	/**
	 * Builds the verifier.
	 *
	 * @param verifier the verifier used for the warnings of each property
	 */
	public SVCOMPVerifier(
			WarningVerifier<A, D> verifier) {
		this.verifier = verifier;
	}

	@Override
	public Map<StatementMessage, VerificationResult> verify(
			SemanticTool<A, D> tool,
			Collection<StatementMessage> warnings)
			throws SemanticException {
		// the warnings that do not report an exception of a property are
		// verified together
		Map<String, List<StatementMessage>> properties = new LinkedHashMap<>();
		for (StatementMessage warning : warnings)
			properties.computeIfAbsent(property(warning), p -> new ArrayList<>()).add(warning);

		Map<StatementMessage, VerificationResult> results = new LinkedHashMap<>();
		for (List<StatementMessage> property : properties.values())
			results.putAll(verifier.verify(tool, property));
		return results;
	}

	// the category of the property violated by the exception reported by the
	// warning, or the empty string if it reports no exception of a property
	private static String property(
			StatementMessage warning) {
		if (!(warning.getViolation() instanceof ErrorViolation))
			return "";
		Type type = ((ErrorViolation) warning.getViolation()).getType();
		for (String exception : ExceptionChecker.PROPERTY_EXCEPTIONS)
			if (type != null && type == PyClassType.lookup(exception))
				return ExceptionChecker.category(exception);
		return "";
	}
}
