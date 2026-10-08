package it.unive.pylisa.checks;

import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.Reachability;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SimpleAbstractDomain;
import it.unive.lisa.analysis.nonrelational.heap.HeapEnvironment;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.value.ValueLattice;
import it.unive.lisa.checks.semantic.SemanticCheck;
import it.unive.lisa.checks.semantic.SemanticTool;
import it.unive.lisa.lattices.ReachabilityProduct;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.lattices.heap.allocations.AllocationSites;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import java.util.ArrayList;
import java.util.List;

/**
 * Assert Checker It checks whether an assertion's condition holds.
 * 
 * @author <a href="mailto:luca.olivieri@unive.it">Luca Olivieri</a>
 */
public class ExceptionsCheck<V extends ValueLattice<V>>
		implements
		SemanticCheck<
				ReachabilityProduct<
						SimpleAbstractState<
								HeapEnvironment<AllocationSites>,
								V,
								TypeEnvironment<TypeSet>>>,
				Reachability<
						SimpleAbstractDomain<
								HeapEnvironment<AllocationSites>,
								V,
								TypeEnvironment<TypeSet>>,
						SimpleAbstractState<
								HeapEnvironment<AllocationSites>,
								V,
								TypeEnvironment<TypeSet>>>> {

	private record VariableInfo(
			String variableName,
			String type,
			String originFile,
			String startLine,
			String assumptionScope) {
	}

	private List<VariableInfo> nonDetVariables = new ArrayList<>();
	// private static final Logger LOG =
	// LogManager.getLogger(ExceptionsCheck.class);

	@Override
	public boolean visit(
			SemanticTool<
					ReachabilityProduct<
							SimpleAbstractState<HeapEnvironment<AllocationSites>, V, TypeEnvironment<TypeSet>>>,
					Reachability<
							SimpleAbstractDomain<
									HeapEnvironment<AllocationSites>,
									V,
									TypeEnvironment<TypeSet>>,
							SimpleAbstractState<
									HeapEnvironment<AllocationSites>,
									V,
									TypeEnvironment<TypeSet>>>> tool,
			CFG graph,
			Statement node) {

		if (node instanceof UnresolvedCall unresolvedCall
				&& unresolvedCall.getQualifier() != null
				&& unresolvedCall.getQualifier().equals("org.sosy_lab.sv_benchmarks.Verifier")) {
			if (unresolvedCall.getParentStatement() instanceof Assignment assignment
					&& unresolvedCall.getTargetName().startsWith("nondet")) {
				String type = unresolvedCall.getTargetName().substring(6);
				String variableName = assignment.getLeft().toString();
				String assumptionScope = "java::LMain;.main([Ljava/lang/String;)V";
				// TODO: we should extract this from the cfg signature.
				if (assignment.getLocation() instanceof SourceCodeLocation sc) {
					String originFile = sc.getSourceFile();
					String startLine = String.valueOf(sc.getLine());

					nonDetVariables.add(new VariableInfo(variableName, type, originFile, startLine, assumptionScope));
				}
			}
		}
		// RuntimeException property checker
		if (graph.getProgram().getEntryPoints().contains(graph) && node instanceof Ret)
			try {
				checkRuntimeException(tool, graph, node);
			} catch (SemanticException e) {
				e.printStackTrace();
			}

		return true;
	}

	private void checkRuntimeException(
			SemanticTool<
					ReachabilityProduct<
							SimpleAbstractState<HeapEnvironment<AllocationSites>, V, TypeEnvironment<TypeSet>>>,
					Reachability<
							SimpleAbstractDomain<HeapEnvironment<AllocationSites>, V, TypeEnvironment<TypeSet>>,
							SimpleAbstractState<HeapEnvironment<AllocationSites>, V, TypeEnvironment<TypeSet>>>> tool,
			CFG graph,
			Statement node)
			throws SemanticException {

		for (var result : tool.getResultOf(graph)) {
			AnalysisState<
					ReachabilityProduct<
							SimpleAbstractState<
									HeapEnvironment<AllocationSites>,
									V,
									TypeEnvironment<TypeSet>>>> state = result.getAnalysisStateAfter(node);

			// checking if there exists at least one exception state
			boolean hasExceptionState = !state.getErrors().isBottom() &&
					!state.getErrors().isTop() &&
					!state.getErrors().function.isEmpty() ||
					(!state.getSmashedErrors().isBottom() &&
							!state.getSmashedErrors().isTop() &&
							!state.getSmashedErrors().function.isEmpty());

			ReachabilityProduct<
					SimpleAbstractState<
							HeapEnvironment<AllocationSites>,
							V,
							TypeEnvironment<TypeSet>>> normaleState = state.getExecutionState();

			// if exceptions had been thrown, we raise a warning
			if (hasExceptionState)
				// if the normal state is bottom, we raise a definite error
				if (normaleState.second.isBottom())
					tool.warnOn((Statement) node, "DEFINITE: uncaught runtime exception in main method");
				// otherwise, we raise a possible error (both normal and
				// exception states are not bottom)
				else
					tool.warnOn((Statement) node, "POSSIBLE: uncaught runtime exception in main method");
		}
	}
}
