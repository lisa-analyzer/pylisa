package it.unive.pylisa.frontend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.cfg.expression.AttributeAccess;
import it.unive.pylisa.cfg.statement.PyCall;
import it.unive.pylisa.frontend.expression.DunderMethods;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Checks that an assignment to a subscript, {@code receiver[key] = value},
 * is translated into a call of {@code __setitem__} that is a node of its
 * function and the parent of the key and the value, and that the receiver's
 * parents lead to it too, so that the errors raised while evaluating any of
 * them lead to that call.
 */
class SubscriptAssignmentTest {

	private static final String PROGRAM = "src/test/resources/programs/subscripts/assign_in_function.py";

	private static final String SEVERAL_INDICES = "src/test/resources/programs/subscripts/assign_several_indices.py";

	@Test
	void theCallOfSetitemIsTheParentOfWhatItEvaluates() throws Exception {
		PyCall write = setitem(PROGRAM);
		Expression[] operands = write.getSubExpressions();
		// the attribute, then the receiver, the key and the value
		assertEquals(4, operands.length, write.toString());
		// the receiver is also the base of the attribute, whose parent is the
		// call: what matters is that every chain of parents reaches the call
		for (int i = 1; i < operands.length; i++)
			assertSame(write, topOf(operands[i]), operands[i] + " belongs to another statement");
		assertSame(write, operands[2].getParentStatement(), "the key");
		assertSame(write, operands[3].getParentStatement(), "the value");
	}

	@Test
	void theValueOfAWriteWithSeveralIndicesIsKept() throws Exception {
		PyCall write = setitem(SEVERAL_INDICES);
		Expression[] operands = write.getSubExpressions();
		assertEquals(4, operands.length, write.toString());
		assertSame(write, operands[3].getParentStatement(), "the value");
	}

	private static PyCall setitem(
			String path)
			throws Exception {
		Program program = new PyFrontend(path, false).toLiSAProgram(true);
		CFG fill = null;
		for (CodeMember member : program.getCodeMembersRecursively())
			if (member instanceof CFG cfg && member.getDescriptor().getFullName().startsWith("__main__.fill::"))
				fill = cfg;
		assertTrue(fill != null, "no function fill in " + program.getCodeMembersRecursively());
		List<PyCall> writes = fill.getNodes().stream()
				.filter(node -> node instanceof PyCall apply
						&& apply.getSubExpressions()[0] instanceof AttributeAccess access
						&& access.getTarget().equals(DunderMethods.SETITEM))
				.map(PyCall.class::cast)
				.toList();
		assertEquals(1, writes.size(), fill.getNodes().toString());
		return writes.get(0);
	}

	private static Statement topOf(
			Expression expression) {
		Statement top = expression;
		while (top instanceof Expression inner && inner.getParentStatement() != null)
			top = inner.getParentStatement();
		return top;
	}
}
