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
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Collections;
import java.util.Set;

/**
 * Python's {@code container[index]} (single-index read; slices go through
 * {@link PyDoubleArrayAccess}). It calls {@code type(container).__getitem__(
 * container, index)} &mdash; the real dunder, so any type registering
 * {@code __getitem__} (the {@code Sequence} hierarchy via
 * {@code SequenceGetItem}, or a user-defined class) is handled the same way,
 * general Python semantics rather than a structure baked into this node. There
 * is no reflected method (indexing is one-directional), so if no runtime type
 * pair resolves it, real Python raises {@code TypeError}; this codebase does
 * not model exceptions, so that is surfaced as
 * {@link UnsupportedStatementException} instead (mirroring {@code in}'s
 * {@code __contains__} dispatch).
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
		Set<Type> rtsContainer = analysis.getRuntimeTypesOf(state, left, this);
		Set<Type> rtsIndex = analysis.getRuntimeTypesOf(state, right, this);
		SymbolAliasing aliasing = state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class);

		AnalysisState<A> result = state.bottom();
		for (Type tContainer : rtsContainer) {
			for (Type tIndex : rtsIndex) {
				if (PyBinaryDispatch.isBuiltinValueType(tContainer)) {
					// str, int, ...: __getitem__ is looked up in their class
					@SuppressWarnings("unchecked")
					Set<Type>[] types = new Set[] { Collections.singleton(tContainer), Collections.singleton(tIndex) };
					Call resolved = PyBinaryDispatch.resolveInClass(interprocedural, state, this,
							PyBinaryDispatch.classOf(tContainer), "__getitem__",
							new Expression[] { getLeft(), getRight() }, types);
					if (resolved == null)
						// e.g. 5[0]: the object is not subscriptable
						result = result.lub(PyExceptions.raise(analysis, state, getCFG(), getLocation(), this,
								LibrarySpecificationProvider.TYPE_ERROR));
					else {
						result = result.lub(resolved.forwardSemanticsAux(interprocedural, state,
								new ExpressionSet[] { new ExpressionSet(left), new ExpressionSet(right) },
								expressions));
						getMetaVariables().addAll(resolved.getMetaVariables());
					}
					continue;
				}

				// type(container).__getitem__(container, index): try both a
				// static-style registration (native types) and an
				// instance-style
				// one (Sequence), matching len()'s dual dispatch
				UnresolvedCall getitem = null;
				for (CallType kind : new CallType[] { CallType.STATIC, CallType.INSTANCE }) {
					UnresolvedCall candidate = new UnresolvedCall(
							getCFG(),
							getLocation(),
							kind,
							null,
							"__getitem__",
							LeftToRightEvaluation.INSTANCE,
							getLeft(),
							getRight());
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
				result = result.lub(getitem.forwardSemantics(state, interprocedural, expressions));
			}
		}

		return result;
	}
}
