package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThatThrownBy;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;

/**
 * Layer A: malformed inputs must surface a parse error — future refactors must
 * keep the ANTLR error-propagation contract.
 */
final class SyntaxErrorReportingTests {

	private static final Logger LOG = LogManager.getLogger(SyntaxErrorReportingTests.class);

	@Test
	void unbalanced_parenthesis_throws() {
		assertThatThrownBy(() -> FrontendTestSupport.parseSnippet("x = (1 + 2\n"))
				.isInstanceOf(Exception.class);
		LOG.debug("unbalanced paren rejected");
	}

	@Test
	void unterminated_string_throws() {
		assertThatThrownBy(() -> FrontendTestSupport.parseSnippet("x = 'unterminated\n"))
				.isInstanceOf(Exception.class);
	}

	@Test
	void reserved_word_as_identifier_throws() {
		assertThatThrownBy(() -> FrontendTestSupport.parseSnippet("def = 1\n"))
				.isInstanceOf(Exception.class);
	}
}
