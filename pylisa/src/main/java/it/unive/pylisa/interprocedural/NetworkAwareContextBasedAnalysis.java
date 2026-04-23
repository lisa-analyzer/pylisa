package it.unive.pylisa.interprocedural;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.AnalyzedCFG;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.network.EndpointKey;
import it.unive.lisa.analysis.network.NetworkAbstractState;
import it.unive.lisa.conf.FixpointConfiguration;
import it.unive.lisa.interprocedural.CFGResults;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.interprocedural.context.KDepthToken;
import it.unive.lisa.lattices.GenericSetLattice;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.util.datastructures.graph.algorithms.FixpointException;
import it.unive.pylisa.cfg.type.PyFunctionType;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Extension of {@link ContextBasedAnalysis} that performs a mandatory second
 * fixpoint pass over endpoint handler functions discovered during the main
 * analysis. Handlers are never called explicitly in the source, so the RTA call
 * graph never reaches them. The second pass uses the main analysis exit state
 * as the entry state (preserving the full heap / network state) and sets
 * handler parameters to {@code unknownValue()} (i.e. TOP) to model arbitrary
 * HTTP input.
 */
public class NetworkAwareContextBasedAnalysis<
		A extends AbstractLattice<A>,
		D extends AbstractDomain<A>>
		extends
		ContextBasedAnalysis<A, D> {

	@Override
	public void fixpoint(
			AnalysisState<A> entryState,
			FixpointConfiguration<A, D> conf)
			throws FixpointException {

		super.fixpoint(entryState, conf);

		AnalysisState<A> mainExitState = computeMainExitState();
		if (mainExitState == null)
			return;

		A execState = mainExitState.getExecutionState();
		if (!(execState instanceof NetworkAbstractState))
			return;

		NetworkAbstractState<?, ?, ?> netState = (NetworkAbstractState<?, ?, ?>) execState;

		Set<String> handlerFullNames = new HashSet<>();
		for (Map.Entry<EndpointKey, GenericSetLattice<String>> entry : netState.handlerMap) {
			GenericSetLattice<String> names = entry.getValue();
			if (names != null && !names.isBottom() && !names.isTop() && names.elements != null)
				handlerFullNames.addAll(names.elements);
		}

		if (handlerFullNames.isEmpty())
			return;

		Set<CFG> handlerCFGs = new HashSet<>();
		for (String fullName : handlerFullNames)
			app.getAllCFGs().stream()
					.filter(cfg -> cfg.getDescriptor().getFullName().equals(fullName))
					.forEach(handlerCFGs::add);

		KDepthToken<A> empty = token.startingId();
		for (CFG handlerCFG : handlerCFGs) {
			try {
				AnalysisState<A> handlerEntry = prepareEntryStateOfEntryPoint(mainExitState, handlerCFG);
				AnalyzedCFG<A> result = handlerCFG.fixpoint(
						handlerEntry, this, conf.fixpointWorkingSet.mk(), conf, empty);
				results.putResult(handlerCFG, empty, result);
			} catch (SemanticException | FixpointException e) {
				throw new FixpointException(
						"Error analyzing endpoint handler " + handlerCFG.getDescriptor().getFullName(), e);
			}
		}
	}

	/**
	 * Computes the lub of all exit states of all entry-point CFGs (i.e. the
	 * {@code $init} function(s)).
	 */
	private AnalysisState<A> computeMainExitState() {
		AnalysisState<A> combined = null;
		for (CFG entryPoint : app.getEntryPoints()) {
			CFGResults<A> cfgResults = results.get(entryPoint);
			if (cfgResults == null)
				continue;
			for (AnalyzedCFG<A> analyzed : cfgResults.getAll()) {
				try {
					AnalysisState<A> exitState = analyzed.getExitState();
					combined = (combined == null) ? exitState : combined.lub(exitState);
				} catch (SemanticException e) {
					// todo: what to do?
				}
			}
		}
		return combined;
	}

	/**
	 * Convenience factory: builds the extractor lambda that resolves a
	 * {@link PyFunctionType} to its handler's full name. Used in
	 * {@link it.unive.lisa.analysis.network.NetworkAwareAbstractDomain} when
	 * constructing the domain instance in tests.
	 */
	public static java.util.function.Function<it.unive.lisa.type.Type, String> pyFunctionNameExtractor() {
		return t -> {
			if (t instanceof PyFunctionType pft) {
				it.unive.lisa.program.cfg.CodeMember cm = pft.getUnit().getFunction();
				if (cm != null) {
					// Use the function-unit name (e.g.
					// "routers.tiles.delete_tile_cache")
					// rather than the CFG descriptor full name which appends
					// "::$call".
					String name = pft.getUnit().getName();
					String loc = cm.getDescriptor().getLocation().getCodeLocation();
					return name + " @ " + loc;
				}
			}
			return null;
		};
	}
}
