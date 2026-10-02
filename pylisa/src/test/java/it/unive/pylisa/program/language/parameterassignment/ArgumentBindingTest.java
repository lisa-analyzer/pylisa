package it.unive.pylisa.program.language.parameterassignment;

import static org.junit.jupiter.api.Assertions.assertEquals;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.Int32Literal;
import it.unive.pylisa.PythonFeatures;
import it.unive.pylisa.PythonTypeSystem;
import it.unive.pylisa.cfg.KeywordOnlyParameter;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Checks how the arguments of a call are bound to the formal parameters of
 * its callee, by position and by name, as Python binds them.
 */
class ArgumentBindingTest {

	private final CFG cfg = new CFG(new CodeMemberDescriptor(PySyntheticLocation.INSTANCE,
			new Program(new PythonFeatures(), new PythonTypeSystem()), false, "caller"));

	@Test
	void positionalArgumentsBindInOrder() {
		assertEquals(Optional.of(List.of(List.of(0), List.of(1))),
				ArgumentBinding.bind(formals(plain("a"), plain("b")), actuals(value(), value())));
	}

	@Test
	void keywordArgumentsBindByName() {
		assertEquals(Optional.of(List.of(List.of(1), List.of(0))),
				ArgumentBinding.bind(formals(plain("a"), plain("b")), actuals(named("b"), named("a"))));
		assertEquals(Optional.of(List.of(List.of(0), List.of(1))),
				ArgumentBinding.bind(formals(plain("a"), plain("b")), actuals(value(), named("b"))));
	}

	@Test
	void aParameterWithoutArgumentTakesItsDefault() {
		assertEquals(Optional.of(List.of(List.of(0), List.of())),
				ArgumentBinding.bind(formals(plain("a"), withDefault("b")), actuals(value())));
	}

	@Test
	void extraArgumentsGoToTheStarredParameters() {
		assertEquals(Optional.of(List.of(List.of(0), List.of(1, 2))),
				ArgumentBinding.bind(formals(plain("a"), rest()),
						actuals(value(), value(), value())));
		assertEquals(Optional.of(List.of(List.of(0), List.of(1))),
				ArgumentBinding.bind(formals(plain("a"), keywords()),
						actuals(value(), named("x"))));
		// *args takes nothing when the positional arguments stop before it
		assertEquals(Optional.of(List.of(List.of(0), List.of())),
				ArgumentBinding.bind(formals(plain("x"), rest()),
						actuals(named("x"))));
		assertEquals(Optional.of(List.of(List.of(0), List.of(), List.of())),
				ArgumentBinding.bind(formals(plain("a"), withDefault("b"),
						rest()), actuals(value())));
		// parameters after * or *args take keywords only
		assertEquals(Optional.of(List.of(List.of(0), List.of(1))),
				ArgumentBinding.bind(formals(plain("a"), new KeywordOnlyParameter(PySyntheticLocation.INSTANCE, "b",
						value())), actuals(value(), named("b"))));
		assertEquals(Optional.of(List.of(List.of(0), List.of(), List.of(1))),
				ArgumentBinding.bind(formals(plain("a"), rest(),
						plain("b")), actuals(value(), named("b"))));
	}

	@Test
	void argumentsThatDoNotMatchHaveNoBinding() {
		// a parameter without argument nor default
		assertEquals(Optional.empty(), ArgumentBinding.bind(formals(plain("a"), plain("b")), actuals(value())));
		// a parameter given twice
		assertEquals(Optional.empty(), ArgumentBinding.bind(formals(plain("a")), actuals(value(), named("a"))));
		// a positional argument for a parameter that takes keywords only
		assertEquals(Optional.empty(), ArgumentBinding.bind(
				formals(keywords()), actuals(value())));
		// an argument too many, and a keyword naming no parameter
		assertEquals(Optional.empty(), ArgumentBinding.bind(formals(plain("a")), actuals(value(), value())));
		assertEquals(Optional.empty(), ArgumentBinding.bind(formals(plain("a")), actuals(value(), named("b"))));
		// a positional argument for a parameter after a bare *
		assertEquals(Optional.empty(), ArgumentBinding.bind(
				formals(plain("a"), new KeywordOnlyParameter(PySyntheticLocation.INSTANCE, "b", value())),
				actuals(value(), value())));
		// a positional argument after a keyword one
		assertEquals(Optional.empty(),
				ArgumentBinding.bind(formals(plain("a"), plain("b")), actuals(named("a"), value())));
	}

	private static Parameter[] formals(
			Parameter... formals) {
		return formals;
	}

	private static Expression[] actuals(
			Expression... actuals) {
		return actuals;
	}

	private static Parameter rest() {
		return new VarPositionalParameter(PySyntheticLocation.INSTANCE, "rest");
	}

	private static Parameter keywords() {
		return new VarKeywordParameter(PySyntheticLocation.INSTANCE, "kw");
	}

	private static Parameter plain(
			String name) {
		return new PyParameter(PySyntheticLocation.INSTANCE, name);
	}

	private Parameter withDefault(
			String name) {
		return new PyParameter(PySyntheticLocation.INSTANCE, name, value());
	}

	private Expression value() {
		return new Int32Literal(cfg, PySyntheticLocation.INSTANCE, 1);
	}

	private Expression named(
			String name) {
		return new NamedParameterExpression(cfg, PySyntheticLocation.INSTANCE, name, value());
	}
}
