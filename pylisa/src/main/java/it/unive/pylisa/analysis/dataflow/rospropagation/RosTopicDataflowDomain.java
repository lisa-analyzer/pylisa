package it.unive.pylisa.analysis.dataflow.rospropagation;

import java.util.Collections;
import java.util.Objects;
import java.util.Set;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.dataflow.DataflowDomain;
import it.unive.lisa.analysis.dataflow.PossibleSet;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;

/**
 * The dataflow domain tracking {@link RosTopic}s created through
 * {@code create_subscription}/{@code create_publisher} calls.
 */
public class RosTopicDataflowDomain extends DataflowDomain<PossibleSet<RosTopic>, RosTopic> {

	@Override
	public PossibleSet<RosTopic> makeLattice() {
		return new PossibleSet<>();
	}

	@Override
	public Set<RosTopic> gen(
			PossibleSet<RosTopic> state,
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		// GEN from assignment (identifier = expression)
		return Collections.emptySet();
	}

	@Override
	public Set<RosTopic> gen(
			PossibleSet<RosTopic> state,
			ValueExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		// GEN from evaluation (non assigning expression)
		if (pp instanceof OpenCall) {
			OpenCall call = (OpenCall) pp;
			if (Objects.equals(call.getTargetName(), "create_subscription")
					|| Objects.equals(call.getTargetName(), "create_publisher"))
				// topic name and message type used by the publisher and
				// subscriber must match to allow them to communicate.
				return Collections.singleton(new RosTopic(null, null, call.getSubExpressions()[2].toString(),
						call.getSubExpressions()[1].toString()));
		}
		return Collections.emptySet();
	}

	@Override
	public Set<RosTopic> kill(
			PossibleSet<RosTopic> state,
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		// KILL from assignment (identifier = expression)
		return Collections.emptySet();
	}

	@Override
	public Set<RosTopic> kill(
			PossibleSet<RosTopic> state,
			ValueExpression expression,
			ProgramPoint pp)
			throws SemanticException {
		// KILL from evaluation (non assigning expression)
		return Collections.emptySet();
	}
}
