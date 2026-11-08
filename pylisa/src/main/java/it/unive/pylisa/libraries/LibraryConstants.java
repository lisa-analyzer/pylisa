package it.unive.pylisa.libraries;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import java.util.Map;

/**
 * The constants of the modules of the standard library that are modeled, with
 * their values in python 3.14:
 * <ul>
 * <li>{@code math.pi}, a {@code float} (as the literal
 * {@code 3.141592653589793});</li>
 * <li>{@code re.IGNORECASE} ({@code 2}) and {@code re.UNICODE} ({@code 32}): in
 * python, they are members of {@code re.RegexFlag}, a subclass of {@code int}
 * whose operations are the ones of {@code int} (e.g.
 * {@code re.IGNORECASE | re.UNICODE}); here, they are {@code int}s (so that
 * only their exact type, e.g. as returned by {@code type}, differs);</li>
 * <li>the strings of the {@code string} module ({@code string.digits},
 * {@code string.ascii_letters}, ...).</li>
 * </ul>
 */
public final class LibraryConstants {

	private LibraryConstants() {
	}

	/**
	 * Yields the expression of the constant {@code module.name}.
	 *
	 * @param cfg      the cfg where the constant is read
	 * @param location the location of the read
	 * @param module   the (dotted) name of the module
	 * @param name     the name of the constant
	 *
	 * @return the literal holding the value of the constant, or {@code null} if
	 *             it is not modeled
	 */
	public static Expression constant(
			CFG cfg,
			CodeLocation location,
			String module,
			String name) {
		switch (module + "." + name) {
		case "math.pi":
			return new Float32Literal(cfg, location, Float.parseFloat("3.141592653589793"));
		case "re.IGNORECASE":
			return new Int32Literal(cfg, location, 2);
		case "re.UNICODE":
			return new Int32Literal(cfg, location, 32);
		default:
			String value = STRINGS.get(module + "." + name);
			return value == null ? null : new PyStringLiteral(cfg, location, value, "'");
		}
	}

	private static final String LOWERCASE = "abcdefghijklmnopqrstuvwxyz";

	private static final String UPPERCASE = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

	private static final String DIGITS = "0123456789";

	private static final String PUNCTUATION = "!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~";

	private static final String WHITESPACE = " \t\n\r\013\014";

	// the constants of the string module
	private static final Map<String, String> STRINGS = Map.of(
			"string.ascii_lowercase", LOWERCASE,
			"string.ascii_uppercase", UPPERCASE,
			"string.ascii_letters", LOWERCASE + UPPERCASE,
			"string.digits", DIGITS,
			"string.hexdigits", DIGITS + "abcdef" + "ABCDEF",
			"string.octdigits", "01234567",
			"string.punctuation", PUNCTUATION,
			"string.whitespace", WHITESPACE,
			"string.printable", DIGITS + LOWERCASE + UPPERCASE + PUNCTUATION + WHITESPACE);
}
