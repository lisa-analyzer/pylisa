package it.unive.pylisa.checks;

import it.unive.lisa.checks.syntactic.CheckTool;
import it.unive.lisa.checks.syntactic.SyntacticCheck;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import java.util.Objects;

public class RosTopicDeclarationFinder implements SyntacticCheck {

	@Override
	public void beforeExecution(CheckTool checkTool) {
	}

	@Override
	public void afterExecution(CheckTool checkTool) {
	}

	@Override
	public boolean visitUnit(CheckTool checkTool, Unit unit) {

		return true;
	}

	@Override
	public void visitGlobal(CheckTool checkTool, Unit unit, Global global, boolean b) {
	}

	@Override
	public boolean visit(CheckTool checkTool, CFG cfg) {
		return true;
	}

	@Override
	public boolean visit(CheckTool checkTool, CFG cfg, Statement statement) {

		if (statement instanceof Call) {
			if (Objects.equals(((Call) statement).getTargetName(), "create_subscription")) {
				String topicName = ((Call) statement).getSubExpressions()[2].toString();
				String messageType = ((Call) statement).getSubExpressions()[1].toString();
				checkTool.warnOn(cfg, "Subscriber found: " + statement + ". Read from topic: " + topicName
						+ ", message type: " + messageType);
			}
			if (Objects.equals(((Call) statement).getTargetName(), "create_publisher")) {
				String topicName = ((Call) statement).getSubExpressions()[2].toString();
				String messageType = ((Call) statement).getSubExpressions()[1].toString();
				checkTool.warnOn(cfg, "Publisher found: " + statement + ". Write on topic: " + topicName
						+ ", message type: " + messageType);
			}
		}
		return true;
	}

	@Override
	public boolean visit(CheckTool checkTool, CFG cfg, Edge edge) {
		return true;
	}
}
