package it.unive.pylisa.frontend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.cfg.CodeMember;
import java.util.Set;
import java.util.TreeSet;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

/**
 * Checks that every construct the frontend translates in a known unsound way,
 * without weakening the analysis, marks the function containing it, so that
 * readers of the results can list it.
 */
class KnownLimitationMarksTest {

	private static final String PROGRAMS = "src/test/resources/programs/limitations/";

	@ParameterizedTest
	@CsvSource(delimiter = '|', value = {
			"nested_argument.py | call nested in the arguments of a call",
			"call_receiver.py | method call on the result of a call",
			"membership_call.py | membership test over a call",
			"for_loop.py | for loop",
			"walrus_if.py | assignment expression in a condition",
			"walrus_while.py | assignment expression in a condition",
			"kwargs.py | unpacked keyword arguments (**kw)",
			"star_args.py | unpacked argument (*args)",
			"generator_argument.py | generator argument",
			"lambda_expression.py | lambda",
			"subscript_on_call.py | method call on the result of a call",
			"subscript_key_call.py | call nested in the arguments of a call",
			"setitem_call.py | call nested in the arguments of a call",
			"star_import_unknown.py | star import (the names it binds are not bound)",
			"star_import_project.py | star import (the names it binds are not bound)",
			"matmul.py | matrix multiplication (@)",
			"class_decorator.py | class decorator not applied",
			"chained_assignment.py | chained assignment (only the first target is assigned)",
			"global_statement.py | global or nonlocal declaration ignored",
			"conditional_call.py | call inside a conditional expression",
			"short_circuit_call.py | call in the right operand of and, or",
			"short_circuit_or_call.py | call in the right operand of and, or",
			"chained_comparison_call.py | call in a chained comparison",
			"chained_membership_call.py | chained comparison with in or is (operands after the second are not evaluated)",
			"adjacent_strings.py | adjacent string literals (only the first is kept)",
			"import_several.py | import of several modules (only the first is imported)" })
	void constructMarksItsFunction(
			String program,
			String construct)
			throws Exception {
		Set<String> marks = marks(PROGRAMS + program);
		assertTrue(marks.contains(construct), program + " is marked with " + marks);
	}

	@Test
	void aCallInTheLeftOperandOfAndOrIsNotMarked() throws Exception {
		// the left operand is evaluated once and its state is stored
		assertEquals(Set.of(), marks(PROGRAMS + "short_circuit_left_call.py"));
	}

	@Test
	void aCallInTheFirstTwoOperandsOfAChainedComparisonIsNotMarked() throws Exception {
		// the first operand is evaluated once, the second once from the state
		// after the first, so both states are stored
		assertEquals(Set.of(), marks(PROGRAMS + "chained_comparison_early_call.py"));
	}

	@Test
	void programWithoutSuchConstructsIsNotMarked() throws Exception {
		assertEquals(Set.of(), marks(PROGRAMS + "none.py"));
	}

	@Test
	void constructTranslatedUnsoundlyKeepsItsMarkNextToALimitation() throws Exception {
		Program translated = new PyFrontend(PROGRAMS + "unsound_and_limitation.py", false).toLiSAProgram(true);
		boolean unsound = false;
		for (CodeMember member : translated.getCodeMembersRecursively())
			for (Annotation annotation : member.getDescriptor().getAnnotations())
				unsound |= annotation.getAnnotationName().equals(ParserSupport.UNSOUND_TRANSLATION);
		assertTrue(unsound, "async def still weakens the analysis");
		assertTrue(marks(PROGRAMS + "unsound_and_limitation.py").contains("for loop"));
	}

	@Test
	void withStatementIsAnUnsoundTranslation() throws Exception {
		// __exit__ is not called, and it may suppress an exception
		Set<String> marks = marks(PROGRAMS + "with_statement.py", ParserSupport.UNSOUND_TRANSLATION);
		assertTrue(marks.contains("with statement (__exit__ may suppress exceptions)"), marks.toString());
	}

	@Test
	void constructTranslatedUnsoundlyIsNamedByItsMark() throws Exception {
		Set<String> marks = marks(PROGRAMS + "unsound_and_limitation.py", ParserSupport.UNSOUND_TRANSLATION);
		assertTrue(marks.contains("async stmt treated as sync"), marks.toString());
	}

	private static Set<String> marks(
			String program)
			throws Exception {
		return marks(program, ParserSupport.KNOWN_LIMITATION);
	}

	private static Set<String> marks(
			String program,
			String kind)
			throws Exception {
		Program translated = new PyFrontend(program, false).toLiSAProgram(true);
		Set<String> constructs = new TreeSet<>();
		for (CodeMember member : translated.getCodeMembersRecursively())
			for (Annotation annotation : member.getDescriptor().getAnnotations())
				if (annotation.getAnnotationName().equals(kind))
					for (AnnotationMember field : annotation.getAnnotationMembers())
						if (field.getId().equals(ParserSupport.CONSTRUCT))
							constructs.add(field.getValue().toString());
		return constructs;
	}
}
