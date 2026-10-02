package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.pylisa.cfg.type.PyExceptionType;
import java.util.Objects;

/**
 * The Python statement {@code raise}, in all its forms.
 * <p>
 * No execution continues normally past the statement: every execution that
 * reaches it, once its sub-expressions are evaluated, becomes an error of the
 * statement's exception type. The type is exact when the raised exception is
 * built by a class of the {@code builtins} module that has a
 * {@link PyExceptionType} (as in {@code raise TypeError("...")}); for any
 * other form ({@code raise e}, a bare {@code raise}, an exception class of the
 * program) it is {@link PyExceptionType#BASE_EXCEPTION}, which stands for any
 * exception.
 * </p>
 * <p>
 * The sub-expressions are those Python evaluates before raising: the
 * arguments of the builtin exception class, or the whole raised expression,
 * and the cause after {@code from}. Their values are discarded, since the
 * raised object is not modelled.
 * </p>
 */
public class PyRaise extends NaryStatement {

	private final PyExceptionType type;

	/**
	 * Builds the statement.
	 *
	 * @param cfg       the CFG the statement belongs to
	 * @param location  the location of the statement
	 * @param type      the type of the raised exception
	 * @param evaluated the expressions evaluated before raising, in order
	 */
	public PyRaise(
			CFG cfg,
			CodeLocation location,
			PyExceptionType type,
			Expression... evaluated) {
		super(cfg, location, "raise", evaluated);
		this.type = Objects.requireNonNull(type);
	}

	/**
	 * Yields the type of the raised exception.
	 *
	 * @return the type
	 */
	public PyExceptionType getExceptionType() {
		return type;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return type.getName().compareTo(((PyRaise) o).type.getName());
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		// a statement has no value: the evaluated expressions are not left
		// behind on the error path
		AnalysisState<A> cleared = analysis.smallStepSemantics(state, new Skip(getLocation()), this);
		return analysis.moveExecutionToError(cleared, new AnalysisState.Error(type, this), this);
	}
}
