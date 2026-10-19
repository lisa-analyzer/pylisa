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
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.type.Type;
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
 * called by the main module): the violation is {@code DEFINITE} if every
 * execution raises that exception at that statement, {@code POSSIBLE}
 * otherwise. An exception of an ancestor of the classes above (e.g.
 * {@code Exception}, raised by code that is not known, such as the one run by
 * {@code exec}) might be any of their subclasses: it is a {@code POSSIBLE}
 * violation of every property.
 */
public class AssertChecker<A extends AbstractLattice<A>, D extends AbstractDomain<A>>
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
				AnalysisState<A> state = result.getAnalysisStateAfter(exit);
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
					// normally, nor raising other exceptions or at other
					// statements
					boolean definite = !terminates && only && throwers.size() == 1;
					String message = (definite ? "DEFINITE" : "POSSIBLE") + ": uncaught " + exception;
					if (thrower == null)
						tool.warnOn(graph, message);
					else
						tool.warnOn(thrower, message);
				}
			}
		}
		return true;
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
