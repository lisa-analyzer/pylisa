package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.CFGThrow;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Set;

/**
 * The {@code raise e} statement (also {@code raise e from cause}), that
 * terminates the execution raising the exception {@code e}. The exception is
 * the error of type {@code e}'s class: an object whose class derives from
 * {@code BaseException}, or whose class has bases (that are not tracked, so the
 * class is assumed to be an exception). Raising any other value raises
 * {@code TypeError}, as python does. The cause is evaluated, and then
 * discarded.
 */
public class PyRaise extends NaryStatement {

	/**
	 * Builds the statement.
	 *
	 * @param cfg       the cfg where the statement is
	 * @param location  the location of the statement
	 * @param exception the raised exception
	 * @param cause     the cause of the exception ({@code from cause}), or
	 *                      {@code null}
	 */
	public PyRaise(
			CFG cfg,
			CodeLocation location,
			Expression exception,
			Expression cause) {
		super(cfg, location, "raise",
				cause == null ? new Expression[] { exception } : new Expression[] { exception, cause });
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public boolean stopsExecution() {
		return true;
	}

	@Override
	public boolean throwsError() {
		return true;
	}

	@Override
	public String toString() {
		Expression[] subs = getSubExpressions();
		return "raise " + subs[0] + (subs.length > 1 ? " from " + subs[1] : "");
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		CompilationUnit base = PyClassType.lookup(LibrarySpecificationProvider.BASE_EXCEPTION).getUnit();

		AnalysisState<A> result = state.bottomExecution();
		boolean typeError = false;
		for (SymbolicExpression e : params[0]) {
			Set<Type> types = analysis.getRuntimeTypesOf(state, e, this);
			for (Type t : types) {
				Type exception = exceptionType(t, base);
				if (exception == null) {
					typeError = true;
					continue;
				}

				CFGThrow thrown = new CFGThrow(getCFG(), exception, loc);
				AnalysisState<A> sem = analysis.assign(state, thrown, e, this);
				// the meta variables are forgotten before moving to the error,
				// as forgetting them later only affects the normal execution
				sem = forgetMetas(sem);
				result = result.lub(analysis.moveExecutionToError(sem.withExecutionExpression(thrown),
						new AnalysisState.Error(exception, this), this));
			}
		}

		if (typeError)
			// TypeError: exceptions must derive from BaseException
			result = result.lub(PyExceptions.raise(analysis, forgetMetas(state), getCFG(), loc, this,
					LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	private <A extends AbstractLattice<A>> AnalysisState<A> forgetMetas(
			AnalysisState<A> state)
			throws SemanticException {
		AnalysisState<A> result = state;
		for (Expression sub : getSubExpressions())
			result = result.forgetIdentifiers(sub.getMetaVariables(), this);
		return result;
	}

	// the class of the exception raised by raising a value of the given type,
	// or null if values of that type are not exceptions
	private static Type exceptionType(
			Type type,
			CompilationUnit base) {
		if (!type.isPointerType() || !type.asPointerType().getInnerType().isUnitType())
			return null;
		Type inner = type.asPointerType().getInnerType();
		CompilationUnit unit = inner.asUnitType().getUnit();
		if (!(unit instanceof ClassUnit))
			return null;
		if (unit.isInstanceOf(base)
				|| unit.getAnnotationList().stream()
						.anyMatch(a -> a.getAnnotationName().equals(PyBinaryDispatch.UNTRACKED_BASES)))
			return inner;
		return null;
	}
}
