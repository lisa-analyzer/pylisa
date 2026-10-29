package it.unive.pylisa.cfg.expression.unary;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.CallSites;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code len(x)} builtin. It invokes {@code type(x).__len__(x)}.
 * {@code __len__} is registered as a static-style method for native types (e.g.
 * {@code str}, mirroring how {@code int}/{@code float} dunders are registered)
 * and as an instance method for {@code Sequence} subtypes ({@code list},
 * {@code set}, {@code dict}, {@code tuple}, {@code slice}), so both call kinds
 * are attempted per runtime type. There is no reflected method, so if
 * {@code __len__} does not resolve either way for a given runtime type, that
 * type simply does not contribute to the result (mirroring the
 * {@code TypeError} Python would raise, which is not explicitly modeled here).
 */
public class PyLength extends it.unive.lisa.program.cfg.statement.UnaryExpression {

	public PyLength(
			CFG cfg,
			CodeLocation location,
			Expression exp) {
		super(cfg, location, "len", exp);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		return length(interprocedural, state, expr, expressions, this);
	}

	/**
	 * Computes {@code len(expr)}, raising errors from {@code thrower} (e.g.
	 * when {@code len} is called through a variable holding it, see
	 * {@link it.unive.pylisa.libraries.Len}).
	 *
	 * @param <A>             the kind of the analysis state
	 * @param <D>             the kind of the abstract domain
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the length is computed
	 * @param expr            the argument
	 * @param expressions     the states of the sub-expressions
	 * @param thrower         the statement raising the errors
	 *
	 * @return the state after the computation
	 *
	 * @throws SemanticException if something goes wrong during the computation
	 */
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> length(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions,
			Statement thrower)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		Set<Type> rts = analysis.getRuntimeTypesOf(state, expr, context, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		for (Type t : rts) {
			if (PyBinaryDispatch.isBuiltinValueType(t)) {
				// str, bytes, int, ...: __len__ is looked up in their class
				Set<Type>[] types = new Set[] { Collections.singleton(t) };
				Call resolved = PyBinaryDispatch.resolveInClass(interprocedural, state, this,
						PyBinaryDispatch.classOf(t), "__len__", new Expression[] { getSubExpression() }, types);
				if (resolved == null)
					// e.g. len(5)
					raising.raise(state, getCFG(), getLocation(), thrower, LibrarySpecificationProvider.TYPE_ERROR);
				else {
					result = result.lub(resolved.forwardSemanticsAux(interprocedural, state,
							new ExpressionSet[] { new ExpressionSet(expr) }, expressions));
					getMetaVariables().addAll(resolved.getMetaVariables());
				}
				continue;
			}

			Call len = null, open = null;
			for (CallType kind : new CallType[] { CallType.STATIC, CallType.INSTANCE }) {
				UnresolvedCall candidate = CallSites.get(interprocedural, this, kind, null, "__len__",
						getSubExpression());
				try {
					Call resolved = interprocedural.resolve(candidate, new Set[] { Collections.singleton(t) },
							aliasing);
					if (resolved instanceof OpenCall)
						// no method found with this call kind (e.g. a
						// __len__ of a class of the program is found as an
						// instance method)
						open = resolved;
					else {
						len = resolved;
						break;
					}
				} catch (CallResolutionException e) {
					// try the next call kind
				}
			}

			if (len == null)
				len = open;
			if (len == null)
				// this type does not support len(): it does not contribute to
				// the result
				continue;
			// the argument has already been evaluated: the call must not
			// evaluate it again (e.g. calling f twice in len(f()))
			result = result.lub(len.forwardSemanticsAux(interprocedural, state,
					new ExpressionSet[] { new ExpressionSet(expr) }, expressions));
			getMetaVariables().addAll(len.getMetaVariables());
		}

		return raising.moveFrom(result);
	}

}
