package it.unive.pylisa.cfg.expression.literal;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.lisa.type.CodeMemberType;
import java.util.Collections;

/**
 * A reference to a function, i.e., the value bound to its name by a {@code def}
 * statement. Its type is the {@link CodeMemberType} of the function, so that
 * the functions a variable might hold are tracked by the type domain (e.g.
 * after {@code say_hello = greet}), and calls through the variable can be
 * resolved (see {@code PyNameCall}).
 */
public class PyFunctionLiteral extends Literal<CodeMember> {

	/**
	 * Builds the literal.
	 *
	 * @param cfg      the cfg where the literal is
	 * @param location the location of the literal
	 * @param function the function
	 */
	public PyFunctionLiteral(
			CFG cfg,
			CodeLocation location,
			CodeMember function) {
		super(cfg, location, function, new CodeMemberType(Collections.singleton(function)));
	}

	@Override
	public String toString() {
		return "<function " + getValue().getDescriptor().getName() + ">";
	}
}
