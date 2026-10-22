package it.unive.pylisa.cfg.expression;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.literal.Literal;
import it.unive.pylisa.PyClassParser;
import it.unive.pylisa.cfg.expression.literal.PyFunctionLiteral;
import it.unive.pylisa.cfg.expression.literal.PyStringLiteral;
import it.unive.pylisa.cfg.expression.literal.PyTypeLiteral;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.WeakHashMap;

/**
 * The class attributes of the classes of a program, that are the names assigned
 * in the body of a class (e.g. {@code v} in {@code class K: v = 3}). The body
 * of a class is not executed by the analysis: the value of a class attribute is
 * known only when it is a constant (an expression made of literals and
 * arithmetic operators) assigned once in the body of the class, and the
 * attribute is never assigned through the class (e.g. {@code K.v = 5}) anywhere
 * in the program.
 */
public final class PyClassAttributes {

	// the operators that can appear in the value of a class attribute
	private static final Set<Class<?>> OPERATORS = Set.of(
			PyNegation.class,
			PyBitwiseNot.class,
			PyAddition.class,
			PySubtraction.class,
			PyMultiplication.class,
			PyDivision.class,
			PyFloorDiv.class,
			PyRemainder.class,
			PyPower.class,
			PyBitwiseAnd.class,
			PyBitwiseOr.class,
			PyBitwiseXor.class,
			PyBitwiseLeftShift.class,
			PyBitwiseRIghtShift.class);

	// for each program, the class attributes assigned through their class
	// (as "Cls.name"), or through an unknown class (as "*.name")
	private static final Map<Program, Set<String>> WRITTEN = Collections.synchronizedMap(new WeakHashMap<>());

	private PyClassAttributes() {
	}

	/**
	 * Clears the caches of this class, so that sequential analyses in the same
	 * JVM do not interfere with each other.
	 */
	public static void clearAll() {
		WRITTEN.clear();
	}

	/**
	 * A class attribute.
	 *
	 * @param value the expression computing the value of the attribute, or
	 *                  {@code null} if the value is unknown
	 */
	public record ClassAttribute(
			Expression value) {
	}

	/**
	 * Whether {@code name} is a method of the given class.
	 *
	 * @param unit the class
	 * @param name the name
	 *
	 * @return whether the class defines a method {@code name}
	 */
	public static boolean isMethod(
			ClassUnit unit,
			String name) {
		return !unit.getInstanceCodeMembersByName(name, true).isEmpty() || !unit.getCodeMembersByName(name).isEmpty();
	}

	/**
	 * Yields the class attribute {@code name} of the given class.
	 *
	 * @param unit the class
	 * @param name the name of the attribute
	 *
	 * @return the attribute, or {@code null} if the class does not assign
	 *             {@code name} in its body
	 */
	public static ClassAttribute attribute(
			ClassUnit unit,
			String name) {
		List<Assignment> writes = new ArrayList<>();
		boolean other = false;
		for (CodeMember body : unit.getCodeMembersByName(PyClassParser.INSTRUMENTED_CLASS_INIT_NAME))
			if (body instanceof CFG)
				for (Statement st : ((CFG) body).getNodes())
					for (Statement inner : flatten(st))
						if (inner instanceof Assignment && targets((Assignment) inner, name))
							if (((Assignment) inner).getLeft() instanceof VariableRef)
								writes.add((Assignment) inner);
							else
								// e.g. v, w = 1, 2
								other = true;
		if (writes.isEmpty() && !other)
			return null;
		if (other || writes.size() != 1 || written(unit, name))
			return new ClassAttribute(null);
		Expression value = writes.get(0).getRight();
		return new ClassAttribute(constant(value) ? value : null);
	}

	// whether the class attribute is assigned through the class somewhere in
	// the program
	private static boolean written(
			ClassUnit unit,
			String name) {
		Set<String> written = WRITTEN.computeIfAbsent(unit.getProgram(), PyClassAttributes::written);
		return written.contains(unit.getName() + "." + name) || written.contains("*." + name);
	}

	private static Set<String> written(
			Program program) {
		Set<String> written = new HashSet<>();
		for (CFG cfg : program.getAllCFGs())
			for (Statement st : cfg.getNodes())
				for (Statement inner : flatten(st))
					if (inner instanceof Assignment)
						for (Expression target : targets(((Assignment) inner).getLeft()))
							classWrite(target, written);
		return written;
	}

	// records target if it is the attribute of a class (Cls.name, or
	// type(o).name and o.__class__.name)
	private static void classWrite(
			Expression target,
			Set<String> written) {
		if (target instanceof UnresolvedCall) {
			// Cls.name is a read of a namespace (see PyStatementParser)
			UnresolvedCall call = (UnresolvedCall) target;
			Expression[] params = call.getParameters();
			if (call.getTargetName().equals("__getattribute__") && params.length == 2
					&& params[1] instanceof PyStringLiteral)
				written.add((params[0] instanceof VariableRef ? ((VariableRef) params[0]).getName() : "*") + "."
						+ ((PyStringLiteral) params[1]).getValue());
		} else if (target instanceof PyAttributeAccess) {
			Expression receiver = ((PyAttributeAccess) target).getSubExpression();
			boolean cls = receiver instanceof PyAttributeAccess
					&& ((PyAttributeAccess) receiver).getName().equals("__class__")
					|| receiver instanceof UnresolvedCall && ((UnresolvedCall) receiver).getTargetName().equals("type");
			if (cls)
				written.add("*." + ((PyAttributeAccess) target).getName());
			else if (receiver instanceof VariableRef)
				// Cls.name, if Cls was not known to be a class when parsing
				written.add(((VariableRef) receiver).getName() + "." + ((PyAttributeAccess) target).getName());
		}
	}

	// whether the assignment assigns the variable name (also inside a tuple)
	private static boolean targets(
			Assignment assignment,
			String name) {
		return targets(assignment.getLeft()).stream()
				.anyMatch(t -> t instanceof VariableRef && ((VariableRef) t).getName().equals(name));
	}

	// the targets of an assignment, that are inside tuples for a, b = ...
	private static List<Expression> targets(
			Expression left) {
		List<Expression> targets = new ArrayList<>();
		if (left instanceof TupleCreation)
			for (Expression e : ((TupleCreation) left).getSubExpressions())
				targets.addAll(targets(e));
		else
			targets.add(left);
		return targets;
	}

	// whether the expression is made only of literals and arithmetic
	// operators
	private static boolean constant(
			Expression e) {
		if (e instanceof Literal)
			return !(e instanceof PyFunctionLiteral) && !(e instanceof PyTypeLiteral);
		if (!OPERATORS.contains(e.getClass()))
			return false;
		for (Expression sub : ((NaryExpression) e).getSubExpressions())
			if (!constant(sub))
				return false;
		return true;
	}

	// the statement and all its sub-expressions
	private static List<Statement> flatten(
			Statement st) {
		List<Statement> all = new ArrayList<>();
		all.add(st);
		Expression[] subs = st instanceof NaryExpression ? ((NaryExpression) st).getSubExpressions()
				: st instanceof NaryStatement ? ((NaryStatement) st).getSubExpressions() : new Expression[0];
		for (Expression sub : subs)
			all.addAll(flatten(sub));
		return all;
	}
}
