package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.value.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.cfg.type.PyLambdaType;
import it.unive.pylisa.symbolic.LambdaConstant;
import java.util.Arrays;
import java.util.List;

public class LambdaExpression extends Expression {
	private final List<Expression> arguments;
	private final Expression body;

	public LambdaExpression(
			List<Expression> arguments,
			Expression body,
			CFG cfg,
			CodeLocation loc) {
		super(cfg, loc, PyLambdaType.INSTANCE);

		this.body = body;
		this.arguments = arguments;
	}

	@Override
	public final int setOffset(
			int offset) {
		this.offset = offset;
		int off = offset;
		for (Expression sub : arguments)
			off = sub.setOffset(off + 1);
		off = body.setOffset(off + 1);
		return off;
	}

	@Override
	public final <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		for (Expression sub : arguments)
			if (!sub.accept(visitor, tool))
				return false;
		if (!body.accept(visitor, tool))
			return false;
		return visitor.visit(tool, getCFG(), this);
	}

	@Override
	public String toString() {
		return "lambda " + Arrays.toString(arguments.toArray()) + " : " + body;
	}

	@Override
	public <A extends AbstractState<A, H, V, T>,
			H extends HeapDomain<H>,
			V extends ValueDomain<V>,
			T extends TypeDomain<T>> AnalysisState<A, H, V, T> semantics(
					AnalysisState<A, H, V, T> state,
					InterproceduralAnalysis<A, H, V, T> interprocedural,
					StatementStore<A, H, V, T> expressions)
					throws SemanticException {
		return state.smallStepSemantics(new LambdaConstant(PyLambdaType.INSTANCE, getLocation(), arguments, body),
				this);
	}
}
