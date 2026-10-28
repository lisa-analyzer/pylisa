package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.InstrumentedReceiver;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Collection;

/**
 * The views {@code d.keys()}, {@code d.values()} and {@code d.items()} of a
 * dictionary {@code d}: objects of the classes {@code DictKeys},
 * {@code DictValues} and {@code DictItems}, whose child {@value #MAPPING} is
 * {@code d}. Since a view reads the dictionary when it is used, it reflects the
 * changes made to the dictionary after its creation. Iterating a view yields
 * the keys, the values or the items of the dictionary (see
 * {@link DictModel#keys}, {@link DictModel#values} and
 * {@link DictModel#items}), and its length is the one of the dictionary. The
 * other operations of views (e.g. {@code in}, or the set operations of
 * {@code keys()}) are not modeled.
 */
public final class DictViews {

	/**
	 * The child of a view holding its dictionary.
	 */
	public static final String MAPPING = "mapping";

	private DictViews() {
	}

	/**
	 * Whether the values of the given type are views of dictionaries.
	 *
	 * @param t the type
	 *
	 * @return whether the values are views
	 */
	public static boolean isView(
			Type t) {
		return kind(t) != null;
	}

	// the name of the class of the views of the given type, or null
	private static String kind(
			Type t) {
		if (!t.isPointerType() || !t.asPointerType().getInnerType().isUnitType())
			return null;
		String name = t.asPointerType().getInnerType().asUnitType().getUnit().getName();
		return name.equals(LibrarySpecificationProvider.DICT_KEYS)
				|| name.equals(LibrarySpecificationProvider.DICT_VALUES)
				|| name.equals(LibrarySpecificationProvider.DICT_ITEMS) ? name : null;
	}

	/**
	 * Creates a view of the dictionary.
	 *
	 * @param analysis the analysis
	 * @param state    the state where the dictionary has been evaluated
	 * @param dict     the dictionary
	 * @param kind     the name of the class of the view
	 * @param pp       the statement creating the view
	 * @param metas    where the receiver of the view is added, as it is a meta
	 *                     variable that the statement must forget
	 *
	 * @return the state where the view has been created
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> create(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression dict,
			String kind,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		CodeLocation loc = pp.getLocation();
		Type type = PyClassType.lookup(kind);
		ReferenceType reftype = new ReferenceType(type);
		InstrumentedReceiver view = new InstrumentedReceiver(reftype, false, loc);
		MemoryAllocation alloc = new MemoryAllocation(type, loc, false);
		AnalysisState<A> current = analysis.assign(Allocations.allocate(analysis, state, alloc, pp), view,
				new HeapReference(reftype, alloc, loc), pp);
		current = analysis.assign(current, mapping(view, type, loc), dict, pp);
		metas.add(view);
		return analysis.smallStepSemantics(current, view, pp);
	}

	/**
	 * Yields the dictionaries of the view.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param view     the view
	 * @param t        the type of the view
	 * @param pp       the statement reading the dictionaries
	 *
	 * @return the state whose computed expressions are the dictionaries
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> dictionaries(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression view,
			Type t,
			Statement pp)
			throws SemanticException {
		return analysis.smallStepSemantics(state, mapping(view, t.asPointerType().getInnerType(), pp.getLocation()),
				pp);
	}

	/**
	 * Yields any of the elements of the view of the given type: the keys, the
	 * values or the items of its dictionaries.
	 *
	 * @param analysis the analysis
	 * @param state    the current state
	 * @param view     the view
	 * @param t        the type of the view
	 * @param pp       the statement iterating over the view
	 * @param metas    where the receivers of the items are added (see
	 *                     {@link DictModel#items})
	 *
	 * @return the state whose computed expressions are the elements
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> elements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression view,
			Type t,
			Statement pp,
			Collection<Identifier> metas)
			throws SemanticException {
		String kind = kind(t);
		AnalysisState<A> dicts = dictionaries(analysis, state, view, t, pp);
		AnalysisState<A> result = state.bottomExecution();
		for (SymbolicExpression dict : dicts.getExecutionExpressions())
			if (kind.equals(LibrarySpecificationProvider.DICT_KEYS))
				result = result.lub(DictModel.keys(analysis, dicts, dict, pp));
			else if (kind.equals(LibrarySpecificationProvider.DICT_VALUES))
				result = result.lub(DictModel.values(analysis, dicts, dict, pp));
			else
				result = result.lub(DictModel.items(analysis, dicts, dict, pp, metas));
		return result;
	}

	private static AccessChild mapping(
			SymbolicExpression view,
			Type type,
			CodeLocation loc) {
		return new AccessChild(Untyped.INSTANCE, new HeapDereference(type, view, loc),
				new Variable(Untyped.INSTANCE, MAPPING, loc), loc);
	}
}
