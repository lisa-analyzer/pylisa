package it.unive.pylisa.cfg.expression;

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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.ResolvedCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Python's {@code container[index]} (a single index, also a slice such as
 * {@code 1:3}; {@link PyDoubleArrayAccess} is only used for
 * {@code base[i, j]} when pandas is loaded). It calls {@code type(container).__getitem__(
 * container, index)} &mdash; the real dunder, so any type registering
 * {@code __getitem__} (the {@code Sequence} hierarchy via
 * {@code SequenceGetItem}, or a user-defined class) is handled the same way,
 * general Python semantics rather than a structure baked into this node. There
 * is no reflected method (indexing is one-directional). Values of builtin
 * classes that do not define {@code __getitem__} (e.g., {@code 5[0]}) and
 * lazy iterators raise {@code TypeError}; if no runtime type pair of an
 * object resolves it, {@link UnsupportedStatementException} is thrown
 * (mirroring {@code in}'s {@code __contains__} dispatch).
 */
public class PySingleArrayAccess extends BinaryExpression {

	public PySingleArrayAccess(
			CFG cfg,
			CodeLocation loc,
			Type staticType,
			Expression receiver,
			Expression index) {
		super(cfg, loc, "[]", staticType, receiver, index);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		return getLeft().toString() + "[" + getRight().toString() + "]";
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		Set<Type> rtsContainer = analysis.getRuntimeTypesOf(state, left, context, this);
		Set<Type> rtsIndex = analysis.getRuntimeTypesOf(state, right, context, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> result = state.bottomExecution();
		for (Type tContainer : rtsContainer) {
			if (Iteration.lazyIterator(tContainer)) {
				// e.g. zip(xs, ys)[0]: the iterator is not subscriptable
				raising.raiseOfTypes(state, left, Set.of(tContainer), rtsContainer, true, getCFG(), getLocation(),
						this, LibrarySpecificationProvider.TYPE_ERROR);
				continue;
			}
			// the executions where the container has this type, computed once
			// for all the types of the index: the call resolved for the type
			// runs only on them (e.g., the natives of str raise their errors
			// only where the container is a str)
			AnalysisState<A> typedContainer = PyBinaryDispatch.ofType(analysis, context, state, left, tContainer,
					rtsContainer, this);
			if (typedContainer.getExecutionState().isBottom())
				continue;
			// the calls already evaluated for this type (the same call is
			// resolved for several types of the index)
			Set<Object> evaluated = new HashSet<>();
			for (Type tIndex : rtsIndex) {
				if (PyBinaryDispatch.isBuiltinValueType(tContainer)) {
					// str, int, ...: __getitem__ is looked up in their class
					Set<Type>[] types = new Set[] { Collections.singleton(tContainer), Collections.singleton(tIndex) };
					Call resolved = PyBinaryDispatch.resolveInClass(interprocedural, state, this,
							PyBinaryDispatch.classOf(tContainer), "__getitem__",
							new Expression[] { getLeft(), getRight() }, types);
					if (resolved == null) {
						// e.g. 5[0]: the object is not subscriptable, raised
						// where the container and the index have these types
						raising.raiseOfTypes(typedContainer, right, Set.of(tIndex), rtsIndex,
								PyBinaryDispatch.checkable(tContainer, rtsContainer), getCFG(), getLocation(), this,
								LibrarySpecificationProvider.TYPE_ERROR);
					} else if (evaluated.add(evaluationKey(resolved))) {
						result = result.lub(resolved.forwardSemanticsAux(interprocedural, typedContainer,
								new ExpressionSet[] { new ExpressionSet(left), new ExpressionSet(right) },
								expressions));
						getMetaVariables().addAll(resolved.getMetaVariables());
					}
					continue;
				}

				// type(container).__getitem__(container, index): try both a
				// static-style registration (native types) and an
				// instance-style one (Sequence), matching len()'s dual dispatch
				UnresolvedCall getitem = null;
				for (CallType kind : new CallType[] { CallType.STATIC, CallType.INSTANCE }) {
					UnresolvedCall candidate = CallSites.get(interprocedural, this, kind, null, "__getitem__",
							getLeft(), getRight());
					try {
						interprocedural.resolve(candidate,
								new Set[] { Collections.singleton(tContainer), Collections.singleton(tIndex) },
								aliasing);
						getitem = candidate;
						break;
					} catch (CallResolutionException e) {
						// try the next call kind
					}
				}

				if (getitem == null)
					// no type implements __getitem__ for this pair: real Python
					// raises TypeError, which is not modeled here
					throw new UnsupportedStatementException(this);
				// the receiver and the index have already been evaluated:
				// evaluating the call would evaluate them again (e.g. calling
				// f twice in f()[0])
				if (evaluated.add(evaluationKey(getitem))) {
					result = result.lub(getitem.forwardSemanticsAux(interprocedural, typedContainer,
							new ExpressionSet[] { new ExpressionSet(left), new ExpressionSet(right) }, expressions));
					getMetaVariables().addAll(getitem.getMetaVariables());
				}
			}
		}

		return raising.moveFrom(result);
	}

	// the key identifying the evaluation of a call (its targets, if it is
	// resolved): calls with the same key evaluated on the same state, with
	// the same arguments, have the same result
	private static Object evaluationKey(
			Call call) {
		return call instanceof ResolvedCall ? new HashSet<>(((ResolvedCall) call).getTargets()) : call;
	}
}
