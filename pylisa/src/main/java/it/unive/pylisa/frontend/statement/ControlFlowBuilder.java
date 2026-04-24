package it.unive.pylisa.frontend.statement;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.code.NodeList;
import it.unive.pylisa.cfg.expression.Break;
import it.unive.pylisa.cfg.expression.Continue;

/**
 * Shared helper for CFG-edge construction patterns used by the statement
 * category visitors introduced in Chunk 4. Centralizes the "fall-through?"
 * predicate and the loop-body rewire pattern so no category visitor has to
 * spell out the {@link Continue}/{@link Break} triad inline.
 */
public final class ControlFlowBuilder {

	private ControlFlowBuilder() {
	}

	/**
	 * A statement stops execution for edge-construction purposes if it does not
	 * fall through: returns, raises, or pylisa's {@code break}/{@code continue}
	 * markers. Consolidates the triad check
	 * {@code stopsExecution() || instanceof Continue || instanceof Break}.
	 *
	 * @param s the statement to probe
	 *
	 * @return {@code true} if the statement does not fall through
	 */
	public static boolean stopsExecution(
			Statement s) {
		return s.stopsExecution() || s instanceof Continue || s instanceof Break;
	}

	/**
	 * Rewires {@link Continue} and {@link Break} statements found inside a loop
	 * body so that they target the supplied continuation and exit nodes instead
	 * of their ANTLR-derived fall-through successors.
	 *
	 * @param block          the edge container holding the loop body
	 * @param bodyStmts      the body statements to scan
	 * @param continueTarget node to route {@code continue} to (the condition)
	 * @param breakTarget    node to route {@code break} to (the loop exit)
	 */
	public static void rewireLoopBody(
			NodeList<CFG, Statement, Edge> block,
			Iterable<Statement> bodyStmts,
			Statement continueTarget,
			Statement breakTarget) {
		for (Statement s : bodyStmts)
			if (s instanceof Continue) {
				for (Edge e : block.getOutgoingEdges(s))
					block.removeEdge(e);
				block.addEdge(new SequentialEdge(s, continueTarget));
			} else if (s instanceof Break) {
				for (Edge e : block.getOutgoingEdges(s))
					block.removeEdge(e);
				block.addEdge(new SequentialEdge(s, breakTarget));
			}
	}
}
