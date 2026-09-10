package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.CFGThrow;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;

/**
 * Shared helper to raise a native Python exception from a library
 * implementation, mirroring how JLiSA raises {@code ArrayIndexOutOfBounds
 * Exception} in {@code JavaArrayAccess}: it allocates an instance of the
 * given (built-in, field-less) exception class, wraps it in a
 * {@link CFGThrow}, and moves the resulting state to the error channel via
 * {@link Analysis#moveExecutionToError(AnalysisState, AnalysisState.Error,
 * it.unive.lisa.program.cfg.ProgramPoint)}. The returned state's normal
 * execution is empty ({@code bottomExecution()}); callers {@code lub} it
 * together with whatever non-exceptional continuation(s) apply.
 */
public final class PyExceptions {

	private PyExceptions() {
	}

	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> raise(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			CFG cfg,
			CodeLocation loc,
			Statement thrower,
			String exceptionClassName)
			throws SemanticException {
		Type exceptionType = PyClassType.lookup(exceptionClassName);

		MemoryAllocation alloc = new MemoryAllocation(exceptionType, loc, false);
		AnalysisState<A> allocState = analysis.smallStepSemantics(state, alloc, thrower);

		AnalysisState<A> exceptionState = state.bottomExecution();
		for (SymbolicExpression th : allocState.getExecutionExpressions()) {
			CFGThrow throwVar = new CFGThrow(cfg, exceptionType, loc);
			AnalysisState<A> tmp = analysis.assign(allocState, throwVar, th, thrower);
			exceptionState = exceptionState.lub(
					analysis.moveExecutionToError(tmp.withExecutionExpression(throwVar),
							new AnalysisState.Error(exceptionType, thrower), thrower));
		}

		return exceptionState;
	}
}
