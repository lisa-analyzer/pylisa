package it.unive.pylisa.cfg.expression;

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
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.dicts.DictModel;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.operators.DictPut;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

/**
 * A dictionary literal {@code {k0: v0, ..., kn: vn}}: a new object is
 * allocated, and the receiver pointing to it (an {@link InstrumentedReceiver})
 * is the value of the literal. The entries are its children (see
 * {@link DictModel}). An entry {@code **m} adds the entries of {@code m} (see
 * {@link DictModel#unpackInto}). When pandas is used, the dictionary is instead
 * a symbolic value, as expected by the dataframe analyses.
 */
public class DictionaryCreation extends NaryExpression {

	// for each entry, whether it is **m (with no key)
	private final boolean[] unpacked;

	/**
	 * Builds the literal.
	 *
	 * @param cfg    the cfg where the literal is
	 * @param loc    the location of the literal
	 * @param values the entries: the key and the value, or {@code null} and the
	 *                   unpacked mapping for {@code **m}
	 */
	@SafeVarargs
	public DictionaryCreation(
			CFG cfg,
			CodeLocation loc,
			Pair<Expression, Expression>... values) {
		super(cfg, loc, "dict", toFlatArray(values));
		this.unpacked = new boolean[values.length];
		for (int i = 0; i < values.length; i++)
			unpacked[i] = values[i].getLeft() == null;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return Arrays.compare(unpacked, ((DictionaryCreation) o).unpacked);
	}

	private static Expression[] toFlatArray(
			Pair<Expression, Expression>[] values) {
		List<Expression> result = new ArrayList<>();
		for (Pair<Expression, Expression> value : values) {
			if (value.getLeft() != null)
				result.add(value.getLeft());
			result.add(value.getRight());
		}
		return result.toArray(Expression[]::new);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			return symbolic(interprocedural, state, params, expressions);

		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();
		Type dictType = PyClassType.lookup(LibrarySpecificationProvider.DICT);
		ReferenceType reftype = new ReferenceType(dictType);
		HeapReference ref = new HeapReference(reftype, new MemoryAllocation(dictType, loc, false), loc);
		InstrumentedReceiver dict = new InstrumentedReceiver(reftype, false, loc);
		AnalysisState<A> current = analysis.assign(state, dict, ref, this);
		HeapDereference deref = new HeapDereference(dictType, dict, loc);
		// the heap tracks the fields accessed on each object, but until the
		// first access to any field in the program, the fields are unknown
		// for all objects (see PyNewObj): accessing __class__, that all
		// objects have, makes the entries of the new dictionary known also
		// when it is empty
		AccessChild cls = new AccessChild(Untyped.INSTANCE, deref, new Variable(Untyped.INSTANCE, "__class__", loc),
				loc);
		current = analysis.smallStepSemantics(current, cls, this)
				.withExecutionExpressions(current.getExecutionExpressions());

		for (int e = 0, i = 0; e < unpacked.length; e++) {
			AnalysisState<A> next = state.bottomExecution();
			if (unpacked[e]) {
				// **m
				for (SymbolicExpression source : params[i])
					next = next.lub(DictModel.unpackInto(analysis, current, dict, source, this, true));
				current = next;
				i++;
				continue;
			}
			for (SymbolicExpression key : params[i])
				for (SymbolicExpression value : params[i + 1])
					if (params[i].size() == 1 && DictModel.trackedKey(key) != null)
						// an entry whose key is tracked
						next = next.lub(analysis.assign(current, new AccessChild(Untyped.INSTANCE, deref,
								DictModel.child(DictModel.trackedKey(key), loc), loc), value, this));
					else
						next = next.lub(DictModel.write(analysis, current, dict, key, value, this));
			current = next;
			i += 2;
		}

		// the keys and the values are consumed here
		for (Expression sub : getSubExpressions())
			current = current.forgetIdentifiers(sub.getMetaVariables(), this);
		getMetaVariables().add(dict);
		return analysis.smallStepSemantics(current, dict, this);
	}

	// the dictionaries used by the dataframe analyses (see
	// DataframeGraphValueDomain)
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> symbolic(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		CodeLocation loc = getLocation();
		for (boolean u : unpacked)
			if (u)
				// the entries of the unpacked mappings are not known
				return interprocedural.getAnalysis().smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc),
						this);
		DictConstant dict = new DictConstant(loc);

		if (params.length == 0)
			return interprocedural.getAnalysis().smallStepSemantics(state, dict, this);

		Type dicttype = PyClassType.lookup(LibrarySpecificationProvider.DICT);
		TernaryOperator append = DictPut.INSTANCE;

		Set<TernaryExpression> ws = new HashSet<>(), tmp = new HashSet<>();
		for (SymbolicExpression firstkey : params[0])
			for (SymbolicExpression firstvalue : params[1])
				ws.add(new TernaryExpression(dicttype, dict, firstkey, firstvalue, append, loc));

		for (int i = 2; i < params.length - 1; i += 2) {
			ExpressionSet key = params[i];
			ExpressionSet value = params[i + 1];

			tmp.addAll(ws);
			ws.clear();
			for (TernaryExpression dicthead : tmp)
				for (SymbolicExpression field : key)
					for (SymbolicExpression init : value)
						ws.add(new TernaryExpression(dicttype, dicthead, field, init, append, loc));
			tmp.clear();
		}

		AnalysisState<A> result = state.bottomExecution();
		for (TernaryExpression completedict : ws)
			result = result.lub(interprocedural.getAnalysis().smallStepSemantics(state, completedict, this));

		return result;
	}
}
