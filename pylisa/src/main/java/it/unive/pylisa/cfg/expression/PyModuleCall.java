package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;

/**
 * The call {@code m.f(args)} of the function {@code f} of a local module
 * {@code m} of the program (also when {@code f} is imported with
 * {@code from m import f}): {@code f} is looked up only among the functions of
 * {@code m}. If {@code m} does not define a function {@code f} (e.g. {@code f}
 * is a variable of the module, assigned to a function of another one), the call
 * is unknown code, that might raise any exception.
 */
public class PyModuleCall extends UnresolvedCall {

	/**
	 * Builds the call.
	 *
	 * @param cfg        the cfg where the call happens
	 * @param location   the location of the call
	 * @param module     the name of the unit of the module
	 * @param name       the name of the function
	 * @param parameters the parameters of the call
	 */
	public PyModuleCall(
			CFG cfg,
			CodeLocation location,
			String module,
			String name,
			Expression... parameters) {
		super(cfg, location, CallType.STATIC, module, name, LeftToRightEvaluation.INSTANCE, parameters);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		AnalysisState<A> result = super.forwardSemanticsAux(interprocedural, state, params, expressions);
		Call resolved;
		try {
			resolved = interprocedural.resolve(this, parameterTypes(expressions, analysis),
					state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
		} catch (CallResolutionException e) {
			throw new SemanticException("Unable to resolve call " + this, e);
		}
		if (resolved instanceof OpenCall) {
			// not a function of the module: unknown code
			PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis);
			raising.raise(state, getCFG(), getLocation(), this, LibrarySpecificationProvider.EXCEPTION);
			return raising.moveFrom(result);
		}
		return result;
	}
}
