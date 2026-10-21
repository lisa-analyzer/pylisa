package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Set;

/**
 * Native implementation of {@code dict.__ior__(self, other)}, that is,
 * {@code self |= other}: the entries of {@code other} are added to
 * {@code self}, overriding the ones it has (see {@link DictModel#unpackInto}).
 * Besides dictionaries, {@code other} might be an iterable of key-value pairs,
 * whose entries are not known; {@code TypeError} might then be raised.
 */
public class DictIOr extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected DictIOr(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static DictIOr build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictIOr(cfg, location, "__ior__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
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
		CodeLocation loc = getLocation();
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			// dictionaries are symbolic values (see DictionaryCreation)
			return analysis.smallStepSemantics(state, left, this);

		Set<Type> types = analysis.getRuntimeTypesOf(state, right, this);
		AnalysisState<A> result;
		if (!types.isEmpty() && types.stream().allMatch(DictModel::isDict))
			result = DictModel.unpackInto(analysis, state, left, right, st, false);
		else
			// e.g. a list of pairs: any entry, or TypeError
			result = DictModel.anyEntry(analysis, state, left, new PushAny(Untyped.INSTANCE, loc), st)
					.lub(PyExceptions.raise(analysis, state, getCFG(), loc, st,
							LibrarySpecificationProvider.TYPE_ERROR));
		// the dictionary itself is the result
		return analysis.smallStepSemantics(result, left, this);
	}
}
