package it.unive.pylisa.libraries;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.literal.Float32Literal;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;

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
 * only their exact type, e.g. as returned by {@code type}, differs).</li>
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
			return null;
		}
	}
}
