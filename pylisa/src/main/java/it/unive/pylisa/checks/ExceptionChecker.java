package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.OptimizedAnalyzedCFG;
import it.unive.lisa.analysis.ProgramState;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.checks.verification.Violation;
import it.unive.lisa.conf.FixpointConfiguration;
import it.unive.lisa.lattices.GenericSetLattice;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * The checker of the properties of the SV-COMP python benchmarks, that are
 * violated if the program terminates raising one of the following exceptions
 * (or an exception of a subclass of them):
 * <ul>
 * <li>assertion safety: {@code AssertionError} (see
 * {@link it.unive.pylisa.cfg.statement.PyAssert});</li>
 * <li>dynamic typing: {@code TypeError}, {@code AttributeError};</li>
 * <li>arithmetic: {@code ZeroDivisionError}, {@code FloatingPointError};</li>
 * <li>data-structure lookups: {@code KeyError}, {@code IndexError}.</li>
 * </ul>
 * The exceptions are the errors of the states at the exit points of the entry
 * points of the program (i.e., the main module). As in JLiSA, a warning is
 * raised on each statement that might raise one of the exceptions above (e.g.
 * the {@code assert} raising {@code AssertionError}, also inside a function
 * called by the main module): the violation is definite if every execution
 * raises that exception at that statement, possible otherwise. As in JLiSA, the
 * warnings also report the violation they refer to (an
 * {@link it.unive.lisa.checks.verification.ErrorViolation} of the exception):
 * the verification of warnings performed after the checks, if a verifier has
 * been selected (see
 * {@link it.unive.lisa.conf.LiSAConfiguration#warningVerifier}), can then
 * confirm or refute them (e.g., the violation at a statement is confirmed if
 * the statement is definitely reachable, and every execution reaching it raises
 * there only that exception). An exception raised by code that is not known
 * (such as the one run by {@code exec}, see
 * {@link LibrarySpecificationProvider#UNKNOWN_EXCEPTION}) might be of any
 * subclass of {@code Exception}: it is a possible violation of every property,
 * and its warnings report no violation to verify. An exception of an ancestor
 * of the classes above (e.g. {@code raise Exception("...")}) is not a
 * violation. The warnings start with the category of the violated property (see
 * {@link #category(String)}), and the possible ones contain {@code might} or
 * {@code possible} (see {@link #message}): their text does not reflect the
 * outcome of their verification.
 */
public class ExceptionChecker<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
		implements
		SemanticCheck<A, D> {

	/**
	 * The exceptions that violate the properties of SV-COMP.
	 */
	public static final List<String> PROPERTY_EXCEPTIONS = List.of(
			LibrarySpecificationProvider.ASSERTION_ERROR,
			LibrarySpecificationProvider.TYPE_ERROR,
			LibrarySpecificationProvider.ATTRIBUTE_ERROR,
			LibrarySpecificationProvider.ZERO_DIVISION_ERROR,
			LibrarySpecificationProvider.FLOATING_POINT_ERROR,
			LibrarySpecificationProvider.KEY_ERROR,
			LibrarySpecificationProvider.INDEX_ERROR);

	@Override
	public boolean visit(
			SemanticTool<A, D> tool,
			CFG graph) {
		if (!graph.getProgram().getEntryPoints().contains(graph))
			return true;

		for (AnalyzedCFG<A> result : tool.getResultOf(graph)) {
			// the exceptions raised at any exit point of the cfg, with the
			// statements raising them
			Map<Type, Set<Statement>> raised = new HashMap<>();
			boolean terminates = false;
			for (Statement exit : graph.getAllExitpoints()) {
				// the errors caught by the except blocks the exit is in (e.g.
				// raised by a raise inside a try) do not end the execution, as
				// for the results of calls (see ContextBasedAnalysis)
				AnalysisState<A> state = tool.getAnalysis().removeCaughtErrors(result.getAnalysisStateAfter(exit),
						exit);
				if (!state.getExecutionState().isBottom())
					terminates = true;
				if (!state.getHalt().isBottom())
					// e.g. sys.exit()
					terminates = true;
				if (!state.getErrors().isBottom() && !state.getErrors().isTop()
						&& state.getErrors().function != null)
					for (Map.Entry<AnalysisState.Error, ProgramState<A>> error : state.getErrors())
						if (!error.getValue().isBottom())
							raised.computeIfAbsent(error.getKey().getType(), t -> new HashSet<>())
									.add(error.getKey().getThrower());
				if (!state.getSmashedErrors().isBottom() && !state.getSmashedErrors().isTop()
						&& state.getSmashedErrors().function != null)
					for (Map.Entry<Type, GenericSetLattice<Statement>> error : state.getSmashedErrors())
						raised.computeIfAbsent(error.getKey(), t -> new HashSet<>())
								.addAll(error.getValue().elements());
			}

			// the statements raising each exception
			Set<Statement> throwers = new HashSet<>();
			raised.values().forEach(throwers::addAll);
			for (String exception : PROPERTY_EXCEPTIONS) {
				PyClassType cls = PyClassType.lookup(exception);
				if (cls == null)
					continue;
				Set<Statement> matching = new HashSet<>();
				Set<Statement> maybe = new HashSet<>();
				boolean only = true;
				for (Map.Entry<Type, Set<Statement>> error : raised.entrySet()) {
					CompilationUnit raisedCls = classOf(error.getKey());
					if (raisedCls != null && raisedCls.isInstanceOf(cls.getUnit()))
						matching.addAll(error.getValue());
					else {
						only = false;
						if (raisedCls != null
								&& raisedCls.getName().equals(LibrarySpecificationProvider.UNKNOWN_EXCEPTION)
								&& cls.getUnit().isInstanceOf(
										PyClassType.lookup(LibrarySpecificationProvider.EXCEPTION).getUnit()))
							// raised by unknown code, that might be of any
							// subclass of Exception (an exception of an
							// ancestor class, as raise Exception("..."), is of
							// that class)
							maybe.addAll(error.getValue());
					}
				}
				Set<Statement> all = new HashSet<>(matching);
				all.addAll(maybe);
				for (Statement thrower : all) {
					// the violation is definite if no execution terminates
					// normally, nor raising other exceptions, and the
					// exception is raised either only at this statement, or
					// by every execution reaching it (e.g. the second
					// statement of x = f() raising TypeError on some paths;
					// 1 + "a", raising it on all the other ones)
					boolean definite = !terminates && only
							&& (throwers.size() == 1 || alwaysRaises(tool, result, thrower));
					boolean unknown = !matching.contains(thrower);
					String message = message(exception, definite, unknown,
							thrower == null ? graph.getDescriptor().getLocation()
									: DerivedLocation.source(thrower.getLocation()));
					if (thrower == null)
						tool.warnOn(graph, message);
					else if (unknown)
						// raising an ancestor of the exception neither
						// confirms nor refutes the violation
						tool.warnOn(thrower, message);
					else
						// the verification of warnings (see
						// LiSAConfiguration#warningVerifier) might confirm
						// or refute the violation, e.g. if the statement is
						// definitely reachable and every execution reaching
						// it raises the exception there. The warning is on
						// the node containing the statement: the post-states
						// of sub-expressions (e.g. the call of x = f()) do not
						// contain the errors they raise, and the verification
						// would refute the violation
						tool.warnOn(node(thrower), Violation.error(cls), message);
				}
			}
		}
		return true;
	}

	// whether every execution reaching the statement raises an exception
	// there, that is, the statement of the cfg containing it never completes
	// normally
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean alwaysRaises(
			SemanticTool<A, D> tool,
			AnalyzedCFG<A> result,
			Statement thrower) {
		Statement root = root(result, thrower);
		if (root == null)
			return false;
		AnalysisState<A> after = stateAfter(tool, result, root);
		return after.getExecutionState().isBottom() && after.getHalt().isBottom();
	}

	// the node of the cfg containing the given statement
	private static Statement node(
			Statement st) {
		Statement root = st instanceof Expression ? ((Expression) st).getRootStatement() : st;
		return root != null && st.getCFG().containsNode(root) ? root : st;
	}

	// the state after the given statement: the results of an optimized
	// fixpoint do not keep the states of all the statements (e.g. of the
	// sub-expressions), that are then computed again from the kept ones
	@SuppressWarnings("unchecked")
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> stateAfter(
			SemanticTool<A, D> tool,
			AnalyzedCFG<A> result,
			Statement st) {
		if (result instanceof OptimizedAnalyzedCFG)
			return ((OptimizedAnalyzedCFG<A, D>) result).getUnwindedAnalysisStateAfter(st,
					new FixpointConfiguration<>(tool.getConfiguration()));
		return result.getAnalysisStateAfter(st);
	}

	// the statement of the cfg containing the given one, if the cfg is the
	// one of the result
	private static <A extends AbstractLattice<A>> Statement root(
			AnalyzedCFG<A> result,
			Statement st) {
		if (st == null)
			return null;
		Statement root = st instanceof Expression ? ((Expression) st).getRootStatement() : st;
		return root != null && result.getNodes().contains(root) ? root : null;
	}

	/**
	 * The warning raised for an exception of a property. It starts with the
	 * category of the property (see {@link #category(String)}), and a possible
	 * violation contains {@code might} or {@code possible}, while a definite
	 * one contains neither.
	 *
	 * @param exception the exception
	 * @param definite  whether the violation is definite
	 * @param unknown   whether the exception is raised by code that is not
	 *                      known (that is, an ancestor of the exception is
	 *                      raised)
	 * @param location  the location where the exception is raised
	 *
	 * @return the message
	 */
	public static String message(
			String exception,
			boolean definite,
			boolean unknown,
			CodeLocation location) {
		String text;
		switch (exception) {
		case LibrarySpecificationProvider.ASSERTION_ERROR:
			text = definite ? "the assertion fails" : "the assertion might fail";
			break;
		case LibrarySpecificationProvider.TYPE_ERROR:
			text = definite ? "a value of the wrong type is used" : "a value of the wrong type might be used";
			break;
		case LibrarySpecificationProvider.ATTRIBUTE_ERROR:
			text = definite ? "an attribute that is not defined is accessed"
					: "an attribute that might not be defined is accessed";
			break;
		case LibrarySpecificationProvider.ZERO_DIVISION_ERROR:
			text = definite ? "division by zero" : "possible division by zero";
			break;
		case LibrarySpecificationProvider.FLOATING_POINT_ERROR:
			text = definite ? "floating-point error" : "possible floating-point error";
			break;
		case LibrarySpecificationProvider.KEY_ERROR:
			text = definite ? "the key is not in the mapping" : "the key might not be in the mapping";
			break;
		default:
			text = definite ? "the index is out of range" : "the index might be out of range";
			break;
		}
		if (unknown)
			// never definite
			text = "code that is not known might raise " + exception;
		return category(exception) + ": " + text + " (uncaught " + exception + ") at " + location;
	}

	/**
	 * The category of the property violated by an exception: {@code ASSERTIONS}
	 * for {@code AssertionError}, {@code TYPING} for {@code TypeError} and
	 * {@code AttributeError}, {@code ARITHMETIC} for {@code ZeroDivisionError}
	 * and {@code FloatingPointError}, {@code LOOKUP} for {@code KeyError} and
	 * {@code IndexError}.
	 *
	 * @param exception the exception, one of {@link #PROPERTY_EXCEPTIONS}
	 *
	 * @return the category
	 */
	public static String category(
			String exception) {
		switch (exception) {
		case LibrarySpecificationProvider.ASSERTION_ERROR:
			return "ASSERTIONS";
		case LibrarySpecificationProvider.TYPE_ERROR:
		case LibrarySpecificationProvider.ATTRIBUTE_ERROR:
			return "TYPING";
		case LibrarySpecificationProvider.ZERO_DIVISION_ERROR:
		case LibrarySpecificationProvider.FLOATING_POINT_ERROR:
			return "ARITHMETIC";
		default:
			return "LOOKUP";
		}
	}

	// the class of the exceptions of the given type, or null if they are not
	// objects of a class
	private static CompilationUnit classOf(
			Type type) {
		if (type.isPointerType())
			type = type.asPointerType().getInnerType();
		if (!type.isUnitType() || !(type.asUnitType().getUnit() instanceof CompilationUnit))
			return null;
		return (CompilationUnit) type.asUnitType().getUnit();
	}
}
