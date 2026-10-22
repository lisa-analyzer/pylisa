package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.ProgramState;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
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
 * raises that exception at that statement, possible otherwise. An exception of
 * an ancestor of the classes above (e.g. {@code Exception}, raised by code that
 * is not known, such as the one run by {@code exec}) might be any of their
 * subclasses: it is a possible violation of every property. The warnings start
 * with the category of the violated property, and the possible ones contain
 * {@code might} or {@code possible} (see {@link #message}).
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
						if (raisedCls != null && cls.getUnit().isInstanceOf(raisedCls))
							// an ancestor of the exception (e.g. Exception,
							// raised by unknown code), that might be any of its
							// subclasses
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
							&& (throwers.size() == 1 || alwaysRaises(result, thrower));
					String message = message(exception, definite, !matching.contains(thrower),
							thrower == null ? graph.getDescriptor().getLocation()
									: DerivedLocation.source(thrower.getLocation()));
					if (thrower == null)
						tool.warnOn(graph, message);
					else
						tool.warnOn(thrower, message);
				}
			}
		}
		return true;
	}

	// whether every execution reaching the statement raises an exception
	// there, that is, the statement of the cfg containing it never completes
	// normally
	private static <A extends AbstractLattice<A>> boolean alwaysRaises(
			AnalyzedCFG<A> result,
			Statement thrower) {
		if (thrower == null)
			return false;
		Statement root = thrower instanceof Expression ? ((Expression) thrower).getRootStatement() : thrower;
		if (root == null || !result.getNodes().contains(root))
			return false;
		AnalysisState<A> after = result.getAnalysisStateAfter(root);
		return after.getExecutionState().isBottom() && after.getHalt().isBottom();
	}

	/**
	 * The warning raised for an exception of a property. It starts with the
	 * category of the property ({@code ASSERTIONS}, {@code TYPING},
	 * {@code ARITHMETIC} or {@code LOOKUP}), and a possible violation contains
	 * {@code might} or {@code possible}, while a definite one contains neither.
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
		String category, text;
		switch (exception) {
		case LibrarySpecificationProvider.ASSERTION_ERROR:
			category = "ASSERTIONS";
			text = definite ? "the assertion fails" : "the assertion might fail";
			break;
		case LibrarySpecificationProvider.TYPE_ERROR:
			category = "TYPING";
			text = definite ? "a value of the wrong type is used" : "a value of the wrong type might be used";
			break;
		case LibrarySpecificationProvider.ATTRIBUTE_ERROR:
			category = "TYPING";
			text = definite ? "an attribute that is not defined is accessed"
					: "an attribute that might not be defined is accessed";
			break;
		case LibrarySpecificationProvider.ZERO_DIVISION_ERROR:
			category = "ARITHMETIC";
			text = definite ? "division by zero" : "possible division by zero";
			break;
		case LibrarySpecificationProvider.FLOATING_POINT_ERROR:
			category = "ARITHMETIC";
			text = definite ? "floating-point error" : "possible floating-point error";
			break;
		case LibrarySpecificationProvider.KEY_ERROR:
			category = "LOOKUP";
			text = definite ? "the key is not in the mapping" : "the key might not be in the mapping";
			break;
		default:
			category = "LOOKUP";
			text = definite ? "the index is out of range" : "the index might be out of range";
			break;
		}
		if (unknown)
			// never definite
			text = "code that is not known might raise " + exception;
		return category + ": " + text + " (uncaught " + exception + ") at " + location;
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
