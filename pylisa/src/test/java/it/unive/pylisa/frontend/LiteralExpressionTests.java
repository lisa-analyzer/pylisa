package it.unive.pylisa.frontend;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Layer A: atomic literal branches of {@code visitAtom}.
 */
final class LiteralExpressionTests {

	private static final Logger LOG = LogManager.getLogger(LiteralExpressionTests.class);

	@Test
	void integer_literal_is_parsed_into_a_program() throws Exception {
		var result = FrontendTestSupport.parseSnippet("x = 42\n");
		LOG.debug("parsed at {}", result.sourceFile());
		assertThat(result.program()).isNotNull();
	}

	@Test
	void hex_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 0xFFFF\n").program()).isNotNull();
	}

	@Test
	void binary_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 0b1010\n").program()).isNotNull();
	}

	@Test
	void octal_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 0o755\n").program()).isNotNull();
	}

	@Test
	void float_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 3.14\n").program()).isNotNull();
	}

	@Test
	void scientific_float_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 1.5e3\n").program()).isNotNull();
	}

	@Test
	void string_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = 'hello'\n").program()).isNotNull();
	}

	@Test
	void triple_quoted_string_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = \"\"\"multi\nline\"\"\"\n").program()).isNotNull();
	}

	@Test
	void raw_string_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = r'\\n'\n").program()).isNotNull();
	}

	@Test
	void bytes_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = b'raw'\n").program()).isNotNull();
	}

	@Test
	void boolean_true_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = True\n").program()).isNotNull();
	}

	@Test
	void boolean_false_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = False\n").program()).isNotNull();
	}

	@Test
	void none_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = None\n").program()).isNotNull();
	}

	@Test
	void ellipsis_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = ...\n").program()).isNotNull();
	}

	@Test
	void fstring_literal_is_parsed() throws Exception {
		assertThat(FrontendTestSupport.parseSnippet("x = f'value={1+2}'\n").program()).isNotNull();
	}

	@ParameterizedTest
	@MethodSource("numericLiterals")
	void numeric_literals_all_parse(
			String snippet)
			throws Exception {
		assertThat(FrontendTestSupport.parseSnippet(snippet).program()).isNotNull();
	}

	private static List<String> numericLiterals() {
		return List.of(
				"x = 0\n", "x = -1\n", "x = 1_000_000\n",
				"x = 0.1\n", "x = .5\n", "x = 1.\n");
	}
}
