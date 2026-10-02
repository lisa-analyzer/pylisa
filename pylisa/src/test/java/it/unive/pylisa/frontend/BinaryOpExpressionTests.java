package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Layer A: every operator dispatch branch in {@code visitComparison}, plus
 * arithmetic, bitwise, boolean, and augmented-assign families.
 */
final class BinaryOpExpressionTests {

	private static final Logger LOG = LogManager.getLogger(BinaryOpExpressionTests.class);

	@ParameterizedTest
	@ValueSource(strings = { "==", "!=", "<", ">", "<=", ">=" })
	void comparison_operators_parse(
			String op)
			throws Exception {
		var result = FrontendTestSupport.parseSnippet("x = 1 " + op + " 2\n");
		LOG.debug("comparison op {} parsed", op);
		assertThat(result.program()).isNotNull();
	}

	@Test
	void is_operator_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a is b\n").program()).isNotNull();
	}

	@Test
	void is_not_operator_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a is not b\n").program()).isNotNull();
	}

	@Test
	void in_operator_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a in b\n").program()).isNotNull();
	}

	@Test
	void not_in_operator_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a not in b\n").program()).isNotNull();
	}

	@ParameterizedTest
	@ValueSource(strings = { "+", "-", "*", "/", "//", "%", "**", "@" })
	void arithmetic_operators_parse(
			String op)
			throws Exception {
		String snippet = "x = a " + op + " b\n";
		assertThat(FrontendTestSupport.parseSnippet(snippet).program()).isNotNull();
	}

	@ParameterizedTest
	@ValueSource(strings = { "&", "|", "^", "<<", ">>" })
	void bitwise_operators_parse(
			String op)
			throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a " + op + " b\n").program()).isNotNull();
	}

	@Test
	void logical_and_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a and b\n").program()).isNotNull();
	}

	@Test
	void logical_or_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = a or b\n").program()).isNotNull();
	}

	@Test
	void logical_not_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = not a\n").program()).isNotNull();
	}

	@Test
	void chained_comparison_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 1 < a < 10\n").program()).isNotNull();
	}

	@ParameterizedTest
	@ValueSource(strings = { "+=", "-=", "*=", "/=", "//=", "%=", "**=", "&=", "|=", "^=", ">>=", "<<=" })
	void augmented_assign_operators_parse(
			String op)
			throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 1\nx " + op + " 2\n").program()).isNotNull();
	}

	@Test
	void unary_minus_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = -y\n").program()).isNotNull();
	}

	@Test
	void unary_plus_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = +y\n").program()).isNotNull();
	}

	@Test
	void bitwise_not_parses() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = ~y\n").program()).isNotNull();
	}
}
