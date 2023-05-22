package it.unive.pylisa.analysis.dataframes;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.HeapAllocationSite;
import it.unive.lisa.analysis.heap.pointbased.StackAllocationSite;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.ObjectRepresentation;
import it.unive.lisa.analysis.representation.SetRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.edge.AssignEdge;
import it.unive.pylisa.analysis.dataframes.edge.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.edge.ConsumeEdge;
import it.unive.pylisa.analysis.dataframes.edge.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.edge.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.operations.AccessOperation;
import it.unive.pylisa.analysis.dataframes.operations.AssignDataframe;
import it.unive.pylisa.analysis.dataframes.operations.AssignValue;
import it.unive.pylisa.analysis.dataframes.operations.BooleanComparison;
import it.unive.pylisa.analysis.dataframes.operations.Concat;
import it.unive.pylisa.analysis.dataframes.operations.CreateFromDict;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.operations.DropColumns;
import it.unive.pylisa.analysis.dataframes.operations.FillNullAxis;
import it.unive.pylisa.analysis.dataframes.operations.FilterNullAxis;
import it.unive.pylisa.analysis.dataframes.operations.Iteration;
import it.unive.pylisa.analysis.dataframes.operations.Keys;
import it.unive.pylisa.analysis.dataframes.operations.ProjectionOperation;
import it.unive.pylisa.analysis.dataframes.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.operations.SelectionOperation;
import it.unive.pylisa.analysis.dataframes.operations.Transform;
import it.unive.pylisa.analysis.dataframes.operations.selection.AtomicBooleanSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.ColumnIteration;
import it.unive.pylisa.analysis.dataframes.operations.selection.ColumnListSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.ColumnRangeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.NumberSlice;
import it.unive.pylisa.analysis.dataframes.operations.selection.RowFilter;
import it.unive.pylisa.analysis.dataframes.operations.selection.Selection;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyLibraryUnitType;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.SliceConstant;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;
import it.unive.pylisa.symbolic.SliceConstant.Slice;
import it.unive.pylisa.symbolic.operators.DictPut;
import it.unive.pylisa.symbolic.operators.ListAppend;
import it.unive.pylisa.symbolic.operators.SliceCreation;
import it.unive.pylisa.symbolic.operators.dataframes.AccessKeys;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRows;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRowsColumns;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation.Kind;
import it.unive.pylisa.symbolic.operators.dataframes.AxisConcatenation;
import it.unive.pylisa.symbolic.operators.dataframes.ColumnAccess;
import it.unive.pylisa.symbolic.operators.dataframes.CopyDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.CreateDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.DataframeColumnSlice;
import it.unive.pylisa.symbolic.operators.dataframes.DataframeColumnSlice.ColumnSlice;
import it.unive.pylisa.symbolic.operators.dataframes.DropCols;
import it.unive.pylisa.symbolic.operators.dataframes.FillNull;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull;
import it.unive.pylisa.symbolic.operators.dataframes.Iterate;
import it.unive.pylisa.symbolic.operators.dataframes.JoinCols;
import it.unive.pylisa.symbolic.operators.dataframes.PandasSeriesComparison;
import it.unive.pylisa.symbolic.operators.dataframes.PopSelection;
import it.unive.pylisa.symbolic.operators.dataframes.ProjectRows;
import it.unive.pylisa.symbolic.operators.dataframes.ReadDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionConstant;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionDataframe;

public class DataframeGraphDomain implements ValueDomain<DataframeGraphDomain> {
	
	private static final Logger LOG = LogManager.getLogger(DataframeGraphDomain.class);

	private static final SetLattice<NodeId> NO_IDS = new SetLattice<NodeId>().bottom();
	private static final SetLattice<DataframeOperation> NO_NODES = new SetLattice<DataframeOperation>().bottom();

	public final ValueEnvironment<ConstantPropagation> constants;

	public final ConstantPropagation constStack;

	public final DataframeForest graph;

	public final CollectingMapLattice<Identifier, NodeId> pointers;

	public final CollectingMapLattice<NodeId, DataframeOperation> operations;

	public DataframeGraphDomain() {
		this.constants = new ValueEnvironment<>(new ConstantPropagation()).top();
		this.graph = new DataframeForest(true).top();
		this.pointers = new CollectingMapLattice<Identifier, NodeId>(new SetLattice<>()).top();
		this.operations = new CollectingMapLattice<NodeId, DataframeOperation>(new SetLattice<>()).top();
		this.constStack = this.constants.lattice.top();
	}

	public DataframeForest getGraph() {
		return graph;
	}

	public ValueEnvironment<ConstantPropagation> getConstants() {
		return constants;
	}

	public CollectingMapLattice<Identifier, NodeId> getPointers() {
		return pointers;
	}

	public CollectingMapLattice<NodeId, DataframeOperation> getOperations() {
		return operations;
	}

	private DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> pointers,
			CollectingMapLattice<NodeId, DataframeOperation> operations) {
		this(constants, constants.lattice.bottom(), graph, pointers, operations);
	}

	private DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			ConstantPropagation constStack,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> pointers,
			CollectingMapLattice<NodeId, DataframeOperation> operations) {
		super();
		this.constants = constants;
		this.constStack = constStack;
		this.graph = graph;
		this.pointers = pointers;

		// cleanup unreachable nodes
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(operations.getMap());
		if (map != null && !map.isEmpty()) {
			Set<NodeId> nodes = new HashSet<>(operations.getKeys());
			for (SetLattice<NodeId> used : this.pointers.getValues())
				used.forEach(nodes::remove);
			pointers.lattice.forEach(nodes::remove);
			nodes.forEach(map::remove);
			this.operations = new CollectingMapLattice<>(operations.lattice, map);
		} else
			this.operations = operations;

		// FIXME temporary sanity check
//		SetLattice<DataframeOperation> pointed = resolvePointers(this);
//		for (DataframeOperation op : pointed)
//			if (!graph.containsNode(op))
//				throw new IllegalStateException();
	}

	@Override
	public DataframeGraphDomain assign(Identifier id, ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		DataframeGraphDomain sss = smallStepSemantics(expression, pp);
		TypeSystem types = pp.getProgram().getTypes();
		if (!sss.constStack.isBottom())
			return new DataframeGraphDomain(
					sss.constants.putState(id, sss.constStack),
					sss.graph,
					sss.pointers,
					sss.operations);
		else if (expression.getRuntimeTypes(types).stream().noneMatch(
				t -> t.toString().equals(LibrarySpecificationProvider.PANDAS_DF)
						|| t.toString().equals(LibrarySpecificationProvider.PANDAS_SERIES)))
			return sss;
		else if (!sss.pointers.lattice.isBottom())
			return new DataframeGraphDomain(
					sss.constants,
					sss.graph,
					sss.pointers.putState(id, sss.pointers.lattice).setStack(sss.pointers.lattice),
					sss.operations);
		else
			return sss;
	}

	@Override
	public DataframeGraphDomain smallStepSemantics(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		if (expression instanceof Identifier)
			return visit((Identifier) expression, pp);
		else if (expression instanceof Constant)
			return visit((Constant) expression, pp);
		else if (expression instanceof PushAny)
			return visit((PushAny) expression, pp);
		else if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			DataframeGraphDomain arg = smallStepSemantics((ValueExpression) unary.getExpression(), pp);
			return arg.visit(unary, arg, pp);
		} else if (expression instanceof BinaryExpression) {
			BinaryExpression binary = (BinaryExpression) expression;
			DataframeGraphDomain left = smallStepSemantics((ValueExpression) binary.getLeft(), pp);
			DataframeGraphDomain right = left.smallStepSemantics((ValueExpression) binary.getRight(), pp);
			return right.visit(binary, left, right, pp);
		} else if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			DataframeGraphDomain left = smallStepSemantics((ValueExpression) ternary.getLeft(), pp);
			DataframeGraphDomain middle = left.smallStepSemantics((ValueExpression) ternary.getMiddle(), pp);
			DataframeGraphDomain right = middle.smallStepSemantics((ValueExpression) ternary.getRight(), pp);
			return right.visit(ternary, left, middle, right, pp);
		}
		return this;
	}

	@Override
	public DataframeGraphDomain assume(ValueExpression expression, ProgramPoint src, ProgramPoint dest)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants.assume(expression, src, dest),
				graph,
				pointers,
				operations);
	}

	@Override
	public DataframeGraphDomain forgetIdentifier(Identifier id) throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> pointers = this.pointers.lift(i -> id.equals(i) ? null : i, e -> e);
		return new DataframeGraphDomain(
				constants.forgetIdentifier(id),
				graph,
				pointers,
				operations.lift(i -> reverseSearch(i, pointers) ? i : null, e -> e));
	}

	private boolean reverseSearch(NodeId id, CollectingMapLattice<Identifier, NodeId> map) {
		for (Entry<Identifier, SetLattice<NodeId>> entry : map)
			if (entry.getValue().contains(id))
				return true;

		return false;
	}

	@Override
	public DataframeGraphDomain forgetIdentifiersIf(Predicate<Identifier> test) throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> pointers = this.pointers.lift(id -> test.test(id) ? null : id, e -> e);
		return new DataframeGraphDomain(
				constants.forgetIdentifiersIf(test),
				graph,
				pointers,
				operations.lift(i -> reverseSearch(i, pointers) ? i : null, e -> e));
	}

	@Override
	public Satisfiability satisfies(ValueExpression expression, ProgramPoint pp) throws SemanticException {
		Satisfiability c = constants.satisfies(expression, pp);
		if (c == Satisfiability.SATISFIED || c == Satisfiability.NOT_SATISFIED)
			return c;
		return Satisfiability.UNKNOWN;
	}

	@Override
	public DataframeGraphDomain pushScope(ScopeToken token) throws SemanticException {
		return this;/*
					 * new DataframeGraphDomain( constants.pushScope(token),
					 * graph, pointers.lift(id -> (Identifier)
					 * id.pushScope(token), e -> e), operations);
					 */
	}

	@Override
	public DataframeGraphDomain popScope(ScopeToken token) throws SemanticException {
		return this;/*
					 * new DataframeGraphDomain( constants.popScope(token),
					 * graph, pointers.lift(id -> (Identifier)
					 * id.popScope(token), e -> e), operations);
					 */
	}

	@Override
	public DomainRepresentation representation() {
		return new ObjectRepresentation(Map.of(
				"constants", constants.representation(),
				"constants-stack", constStack.representation(),
				"pointers", pointers.representation(StringRepresentation::new),
				"pointers-stack", new SetRepresentation(pointers.lattice.elements(), StringRepresentation::new),
				"operations", operations.representation(StringRepresentation::new),
				"graph", graph.representation()));
	}

	@Override
	public DataframeGraphDomain lub(DataframeGraphDomain other) throws SemanticException {
		return new DataframeGraphDomain(
				constants.lub(other.constants),
				constStack.lub(other.constStack),
				graph.lub(other.graph),
				pointers.lub(other.pointers),
				operations.lub(other.operations));
	}

	@Override
	public DataframeGraphDomain widening(DataframeGraphDomain other) throws SemanticException {
		return new DataframeGraphDomain(
				constants.widening(other.constants),
				constStack.widening(other.constStack),
				graph.widening(other.graph),
				pointers.widening(other.pointers),
				operations.widening(other.operations));
	}

	@Override
	public boolean lessOrEqual(DataframeGraphDomain other) throws SemanticException {
		return constants.lessOrEqual(other.constants)
				&& constStack.lessOrEqual(other.constStack)
				&& graph.lessOrEqual(other.graph)
				&& pointers.lessOrEqual(other.pointers)
				// functional lattice does not check the partial order over the
				// inner lattice instance
				&& pointers.lattice.lessOrEqual(other.pointers.lattice)
				&& operations.lessOrEqual(other.operations);
	}

	@Override
	public DataframeGraphDomain top() {
		return new DataframeGraphDomain(
				constants.top(),
				constStack.top(),
				graph.top(),
				pointers.top(),
				operations.top());
	}

	@Override
	public boolean isTop() {
		return constants.isTop() && constStack.isTop() && graph.isTop() && pointers.isTop() && operations.isTop();
	}

	@Override
	public DataframeGraphDomain bottom() {
		return new DataframeGraphDomain(
				constants.bottom(),
				constStack.bottom(),
				graph.bottom(),
				pointers.bottom(),
				operations.bottom());
	}

	@Override
	public boolean isBottom() {
		return constants.isBottom()
				&& constStack.isBottom()
				&& graph.isBottom()
				&& pointers.isBottom()
				&& operations.isBottom();
	}

	public static boolean isDataframeRelated(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes(null).stream()
						.anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, false))
				: PyLibraryUnitType.is(expression.getStaticType(), LibrarySpecificationProvider.PANDAS, false)
						|| expression.getStaticType().isUntyped();
	}

	private static boolean topOrBottom(Lattice<?> l) {
		return l.isTop() || l.isBottom();
	}

	private static SetLattice<DataframeOperation> resolvePointers(DataframeGraphDomain domain) {
		return resolvePointers(domain.operations, domain.pointers.lattice);
	}

	private static SetLattice<DataframeOperation> resolvePointers(
			CollectingMapLattice<NodeId, DataframeOperation> operations,
			SetLattice<NodeId> ids) {
		if (topOrBottom(operations) || topOrBottom(ids))
			return NO_NODES;

		Set<DataframeOperation> resolved = new HashSet<>();
		for (NodeId pointer : ids)
			resolved.addAll(operations.getState(pointer).elements());
		return new SetLattice<>(resolved, false);
	}

	private static SetLattice<DataframeOperation> moveBackwards(
			DataframeForest forest,
			SetLattice<DataframeOperation> ops) {
		if (topOrBottom(ops))
			return ops;

		Set<DataframeOperation> preds = new HashSet<>();
		for (DataframeOperation op : ops)
			preds.addAll(forest.predecessorsOf(op));
		return new SetLattice<>(preds, false);
	}

	/**
	 * Yields a copy of pointers, with the same lattice, but where stack has
	 * been replaced with ids in all mappings.
	 */
	private static CollectingMapLattice<Identifier, NodeId> shift(
			CollectingMapLattice<Identifier, NodeId> pointers,
			SetLattice<NodeId> stack,
			SetLattice<NodeId> ids) {
		if (stack.isTop())
			return pointers.top();
		if (stack.isBottom())
			return pointers.bottom();

		HashMap<Identifier, SetLattice<NodeId>> map = new HashMap<>(pointers.getMap());
		for (Entry<Identifier, SetLattice<NodeId>> entry : pointers)
			if (entry.getValue().intersects(stack))
				map.put(entry.getKey(), entry.getValue().replace(stack, ids));

		return new CollectingMapLattice<>(pointers.lattice, map);
	}

	private static RangeBound getRangeBound(ConstantPropagation c) {
		RangeBound bound;
		if (c.isTop())
			bound = null;
		else if (c.is(Integer.class))
			bound = new RangeBound(c.as(Integer.class));
		else
			bound = c.as(RangeBound.class);
		return bound;
	}

	private static ColumnRangeSelection getRangeBound(SetLattice<DataframeOperation> ops) throws SemanticException {
		ColumnRangeSelection bound = ColumnRangeSelection.BOTTOM;

		for (DataframeOperation op : ops) {
			if (!(op instanceof AccessOperation<?>))
				return ColumnRangeSelection.TOP;

			Selection<?> selection = ((AccessOperation<?>) op).getSelection();
			if (!(selection instanceof ColumnRangeSelection))
				return ColumnRangeSelection.TOP;

			bound = bound.lub((ColumnRangeSelection) selection);
		}

		return bound;
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeOperation toProjection(
			DataframeGraphDomain domain,
			DataframeForest forest,
			Map<NodeId, SetLattice<DataframeOperation>> finalOperations,
			DataframeOperation access) {
		DataframeOperation tmp = new ProjectionOperation(access.getWhere(),
				((AccessOperation) access).getSelection());
		forest.replace(access, tmp);
		for (Entry<NodeId, SetLattice<DataframeOperation>> entry : domain.operations)
			if (entry.getValue().contains(access))
				finalOperations.put(entry.getKey(), finalOperations.get(entry.getKey()).replace(access, tmp));
		return tmp;
	}

	public DataframeGraphDomain visit(UnaryExpression expression, DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		if (arg.isBottom())
			return arg;

		UnaryOperator operator = expression.getOperator();
		if (operator == ReadDataframe.INSTANCE)
			return doReadDataframe(arg, pp);
		if (operator == CreateDataframe.INSTANCE)
			return doCreateDataframe(arg, pp);
		else if (operator == CopyDataframe.INSTANCE)
			return doCopyDataframe(arg, pp);
		else if (operator instanceof ApplyTransformation)
			return doApplyTransformation(arg, operator, pp);
		else if (operator instanceof FilterNull)
			return doFilterNull(arg, operator, pp);
		else if (operator instanceof PopSelection)
			return doPopSelection(arg, pp);
		else if (operator instanceof AxisConcatenation)
			return doAxisConcatenation(arg, operator, pp);
		else if (operator == AccessKeys.INSTANCE)
			return doAccessKeys(arg, pp);
		else if (operator == Iterate.INSTANCE)
			return doIterate(arg, pp);
		else if (!arg.constStack.isBottom()
				&& arg.constants.lattice.canProcess(expression))
			return delegateToConstants(expression, arg, pp);
		else
			return cleanStack(arg, pp);
	}

	private static DataframeGraphDomain delegateToConstants(ValueExpression expression, DataframeGraphDomain arg,
			ProgramPoint pp) throws SemanticException {
		return new DataframeGraphDomain(
				arg.constants,
				arg.constStack.eval(expression, arg.constants, pp),
				arg.graph,
				arg.pointers.setStack(NO_IDS),
				arg.operations);
	}

	private static DataframeGraphDomain doAccessKeys(DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(arg);
		if (topOrBottom(ops))
			return cleanStack(arg, pp);

		Keys access = new Keys(pp.getLocation());
		DataframeForest forest = new DataframeForest(arg.graph);
		forest.addNode(access);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, access));

		NodeId id = new NodeId(access);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, ids);
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				pointers.setStack(ids),
				arg.operations.putState(id, new SetLattice<>(access)));
	}

	private static DataframeGraphDomain doIterate(DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(arg);
		if (topOrBottom(ops))
			return cleanStack(arg, pp);

		Iteration access = new Iteration(pp.getLocation());
		DataframeForest forest = new DataframeForest(arg.graph);
		forest.addNode(access);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, access));

		NodeId id = new NodeId(access);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, ids);
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				pointers.setStack(ids),
				arg.operations.putState(id, new SetLattice<>(access)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doAxisConcatenation(DataframeGraphDomain arg, UnaryOperator operator,
			ProgramPoint pp)
			throws SemanticException {
		ConstantPropagation list = arg.constStack;
		if (topOrBottom(list) || !list.is(List.class))
			return cleanStack(arg, pp);

		List<Lattice<?>> elements = list.as(List.class);
		if (elements.isEmpty())
			return cleanStack(arg, pp);

		Lattice<?> firstWrapped = elements.iterator().next();
		if (elements.size() == 1)
			if (firstWrapped instanceof SetLattice<?>)
				return new DataframeGraphDomain(
						arg.constants,
						arg.graph,
						arg.pointers.setStack((SetLattice<NodeId>) firstWrapped),
						arg.operations);
			else
				return cleanStack(arg, pp);

		DataframeForest forest = new DataframeForest(arg.graph);
		DataframeOperation concatNode = new Concat(pp.getLocation(),
				operator == JoinCols.INSTANCE ? Concat.Axis.CONCAT_COLS : Concat.Axis.CONCAT_ROWS);
		forest.addNode(concatNode);

		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.operations.getMap());
		Map<DataframeOperation, DataframeOperation> replacements = new HashMap<>();

		// 1) if a leaf is an access it has to become a projection
		// 2) the same leaf might appear multiple times as different
		// nodes of the concat

		int nelements = elements.size();
		List<Lattice<?>> distinctElements = elements.stream().distinct().collect(Collectors.toList());
		int[] opIndexes = new int[nelements];
		Map<Integer, List<Integer>> positions = new HashMap<>();
		for (int i = 0; i < nelements; i++) {
			int idx = distinctElements.indexOf(elements.get(i));
			opIndexes[i] = idx;
			positions.computeIfAbsent(idx, index -> new LinkedList<>()).add(i);
		}

		SetLattice<DataframeOperation>[] operands = new SetLattice[nelements];
		for (Entry<Integer, List<Integer>> entry : positions.entrySet()) {
			Lattice<?> element = distinctElements.get(entry.getKey());
			if (!(element instanceof SetLattice<?>))
				return cleanStack(arg, pp);

			SetLattice<DataframeOperation> ops = resolvePointers(arg.operations, (SetLattice<NodeId>) element);
			List<Integer> value = entry.getValue();
			if (value.size() == 1)
				// appears once
				operands[value.iterator().next()] = ops;
			else {
				// appears more than once
				Set<DataframeOperation> operand = ops.elements();
				FIFOWorkingSet<SetLattice<DataframeOperation>> ws = FIFOWorkingSet.mk();
				for (int i = 0; i < value.size(); i++) {
					Set<DataframeOperation> fixed = new HashSet<>();
					for (DataframeOperation op : operand) {
						if (op instanceof AccessOperation<?>)
							if (replacements.containsKey(op)) {
								op = replacements.get(op);
							} else {
								DataframeOperation tmp = toProjection(arg, forest, operations, op);
								replacements.put(op, tmp);
								op = tmp;
							}
						fixed.add(op);
					}

					ws.push(new SetLattice<>(fixed, false));
					operand = fixed.stream().flatMap(o -> forest.predecessorsOf(o).stream()).distinct()
							.collect(Collectors.toSet());
				}

				for (int pos = value.size() - 1; pos >= 0; pos--)
					operands[value.get(pos)] = ws.pop();
			}
		}

		for (int i = 0; i < operands.length; i++) {
			SetLattice<DataframeOperation> ops = operands[i];
			for (DataframeOperation exit : ops)
				forest.addEdge(new ConcatEdge(exit, concatNode, i));
		}

		NodeId id = new NodeId(concatNode);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(arg.operations.lattice, operations);
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				arg.pointers.setStack(new SetLattice<>(id)),
				ops.putState(id, new SetLattice<>(concatNode)));
	}

	private static DataframeGraphDomain doPopSelection(DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(arg);
		if (topOrBottom(ops))
			return cleanStack(arg, pp);

		DataframeForest forest = new DataframeForest(arg.graph);
		Set<NodeId> ids = new HashSet<>();
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.operations.getMap());
		for (DataframeOperation op : ops) {
			if (!(op instanceof SelectionOperation<?>))
				return cleanStack(arg, pp);

			Collection<DataframeOperation> preds = forest.predecessorsOf(op);
			if (preds.size() != 1)
				throw new SemanticException("Not supported yet");
			DataframeOperation pred = preds.iterator().next();
			forest.getNodeList().removeNode(op);
			for (Entry<NodeId, SetLattice<DataframeOperation>> entry : arg.operations)
				if (entry.getValue().contains(op)) {
					operations.put(entry.getKey(), entry.getValue().replace(op, pred));
					ids.add(entry.getKey());
				} else if (entry.getValue().contains(pred))
					ids.add(entry.getKey());
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids, false);
		CollectingMapLattice<Identifier,
				NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, idsLattice);
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				pointers.setStack(idsLattice),
				new CollectingMapLattice<>(arg.operations.lattice, operations));
	}

	private static DataframeGraphDomain doFilterNull(DataframeGraphDomain arg, UnaryOperator operator, ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(arg);
		if (topOrBottom(ops))
			return cleanStack(arg, pp);

		FilterNullAxis filter = new FilterNullAxis(pp.getLocation(), ((FilterNull) operator).getAxis());

		DataframeForest forest = new DataframeForest(arg.graph);
		forest.addNode(filter);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, filter));

		NodeId id = new NodeId(filter);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, ids);
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				pointers.setStack(ids),
				arg.operations.putState(id, new SetLattice<>(filter)));
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doApplyTransformation(DataframeGraphDomain arg, UnaryOperator operator,
			ProgramPoint pp) throws SemanticException {
		SetLattice<DataframeOperation> stack = resolvePointers(arg);
		if (topOrBottom(stack))
			return cleanStack(arg, pp);

		ApplyTransformation op = (ApplyTransformation) operator;
		Kind kind = op.getKind();

		DataframeForest df = new DataframeForest(arg.graph);
		Map<NodeId, DataframeOperation> ids = new HashMap<>();
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(arg.operations.getMap());
		for (DataframeOperation leaf : stack) {
			if (!(leaf instanceof SelectionOperation<?>))
				return cleanStack(arg, pp);

			Transform t = op.getArg().isPresent()
					? new Transform(pp.getLocation(), kind, op.isChangeShape(),
							((SelectionOperation<?>) leaf).getSelection(), op.getArg().get())
					: new Transform(pp.getLocation(), kind, op.isChangeShape(),
							((SelectionOperation<?>) leaf).getSelection());

			df.addNode(t);
			df.addEdge(new ConsumeEdge(leaf, t));
			NodeId id = new NodeId(t);
			ids.put(id, t);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(arg.operations.lattice, map);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, idsLattice);
		return new DataframeGraphDomain(
				arg.constants,
				df,
				pointers.setStack(idsLattice),
				ops);
	}

	private static DataframeGraphDomain doCopyDataframe(DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		if (topOrBottom(arg.pointers.lattice))
			return cleanStack(arg, pp);

		Set<NodeId> ids = new HashSet<>();
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.operations.getMap());
		for (NodeId id : arg.pointers.lattice) {
			NodeId copy = new NodeId(id);
			ids.add(copy);
			operations.put(copy, arg.operations.getState(id));
		}
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				arg.graph,
				arg.pointers.setStack(new SetLattice<>(ids, false)),
				new CollectingMapLattice<>(arg.operations.lattice, operations));
	}

	private static DataframeGraphDomain doReadDataframe(DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		ConstantPropagation filename = arg.constStack;
		if (topOrBottom(filename))
			return cleanStack(arg, pp);

		DataframeForest df = new DataframeForest(arg.graph);
		ReadFromFile op = new ReadFromFile(pp.getLocation(), filename.as(String.class));
		df.addNode(op);

		NodeId id = new NodeId(op);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		SetLattice<DataframeOperation> operations = new SetLattice<>(op);
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				df,
				arg.pointers.setStack(ids),
				arg.operations.putState(id, operations));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doCreateDataframe(DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		ConstantPropagation dict = arg.constStack;
		if (topOrBottom(dict))
			return cleanStack(arg, pp);

		DataframeForest df = new DataframeForest(arg.graph);
		Map<Lattice<?>, Lattice<?>> c;
		if (!dict.is(Map.class))
			c = Map.of();
		else
			c = dict.as(Map.class);
		Set<String> cols = new HashSet<>();
		for (Lattice<?> name : c.keySet())
			if (!topOrBottom(name) && name instanceof ConstantPropagation
					&& ((ConstantPropagation) name).is(String.class))
				cols.add(((ConstantPropagation) name).as(String.class));
		CreateFromDict op = new CreateFromDict(pp.getLocation(), new Names(cols));
		df.addNode(op);

		NodeId id = new NodeId(op);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		SetLattice<DataframeOperation> operations = new SetLattice<>(op);
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				df,
				arg.pointers.setStack(ids),
				arg.operations.putState(id, operations));
	}

	public DataframeGraphDomain visit(BinaryExpression expression, DataframeGraphDomain left,
			DataframeGraphDomain right, ProgramPoint pp) throws SemanticException {
		if (left.isBottom())
			return left;
		if (right.isBottom())
			return right;

		BinaryOperator operator = expression.getOperator();

		if (operator == ListAppend.INSTANCE)
			return doListAppend(left, right, pp);
		else if (operator == ColumnAccess.INSTANCE)
			return doColumnAccess(left, right, pp);
		else if (operator instanceof FillNull)
			return doFillNull(left, right, operator, pp);
		else if (operator == DropCols.INSTANCE)
			return doDropColumns(left, right, pp);
		else if (operator == JoinCols.INSTANCE)
			return doJoinColumns(left, right, pp);
		else if (operator instanceof WriteSelectionDataframe)
			return doWriteSelectionDataframe(left, right, pp);
		else if (operator instanceof WriteSelectionConstant)
			return doWriteSelectionConstant(left, right, pp);
		else if (operator instanceof PandasSeriesComparison)
			return doPandasSeriesComparison(left, right, pp, operator);
		else if (!left.constStack.isBottom()
				&& !right.constStack.isBottom()
				&& right.constants.lattice.canProcess(expression))
			return delegateToConstants(expression, right, pp);
		else
			return cleanStack(right, pp);
	}

	private static DataframeGraphDomain doPandasSeriesComparison(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp, BinaryOperator operator) throws SemanticException {
		SetLattice<DataframeOperation> df1 = resolvePointers(left);
		ConstantPropagation value = right.constStack;
		if (topOrBottom(df1))
			return cleanStack(right, pp);
		if (value.isBottom()) // unknown variables can lead to top
			value = value.top();

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
		PandasSeriesComparison seriesCompOp = (PandasSeriesComparison) operator;
		Map<NodeId, DataframeOperation> ids = new HashMap<>();

		for (DataframeOperation op : df1) {
			if (!(op instanceof SelectionOperation))
				return cleanStack(right, pp);

			SelectionOperation<?> projection = (SelectionOperation<?>) op;
			if (!(projection.getSelection() instanceof ColumnListSelection))
				return cleanStack(right, pp);

			AtomicBooleanSelection booleanSelection = new AtomicBooleanSelection(
					(ColumnListSelection) projection.getSelection(), seriesCompOp.getOp(), value);
			BooleanComparison<
					AtomicBooleanSelection> boolComp = new BooleanComparison<>(pp.getLocation(),
							booleanSelection);

			forest.addNode(boolComp);
			forest.addEdge(new ConsumeEdge(op, boolComp));
			NodeId id = new NodeId(boolComp);
			ids.put(id, boolComp);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, idsLattice);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(idsLattice),
				ops);
	}

	private static DataframeGraphDomain doWriteSelectionConstant(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp) throws SemanticException {
		SetLattice<DataframeOperation> df = resolvePointers(left);
		ConstantPropagation c = right.constStack;
		if (topOrBottom(df) || topOrBottom(c))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
		Map<NodeId, DataframeOperation> ids = new HashMap<>();

		for (DataframeOperation op : df) {
			if (!(op instanceof SelectionOperation))
				return cleanStack(right, pp);

			SelectionOperation<?> access = (SelectionOperation<?>) op;
			if (!(access.getSelection() instanceof DataframeSelection))
				return cleanStack(right, pp);

			DataframeSelection<?, ?> selection = (DataframeSelection<?, ?>) access.getSelection();
			DataframeOperation nodeToAdd = new AssignValue<>(pp.getLocation(), selection, c);
			forest.addNode(nodeToAdd);
			forest.addEdge(new ConsumeEdge(op, nodeToAdd));
			NodeId id = new NodeId(nodeToAdd);
			ids.put(id, nodeToAdd);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, idsLattice);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(idsLattice),
				ops);
	}

	@SuppressWarnings({ "rawtypes", "unchecked" })
	private static DataframeGraphDomain doWriteSelectionDataframe(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp) throws SemanticException {
		SetLattice<DataframeOperation> df1 = resolvePointers(left);
		SetLattice<DataframeOperation> df2 = resolvePointers(right);
		if (topOrBottom(df1) || topOrBottom(df2))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
		Map<NodeId, DataframeOperation> ids = new HashMap<>();

		for (DataframeOperation op : df1) {
			if (!(op instanceof SelectionOperation<?>))
				return cleanStack(right, pp);

			AssignDataframe assign = new AssignDataframe(pp.getLocation(),
					((SelectionOperation) op).getSelection());
			forest.addNode(assign);
			forest.addEdge(new ConsumeEdge(op, assign));
			NodeId id = new NodeId(assign);
			ids.put(id, assign);
		}

		for (DataframeOperation op : df2) {
			if (op instanceof AccessOperation<?>)
				op = toProjection(right, forest, operations, op);

			for (DataframeOperation assign : ids.values())
				forest.addEdge(new AssignEdge(op, assign));
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, idsLattice);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(idsLattice),
				ops);
	}

	private static DataframeGraphDomain doJoinColumns(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> df1 = resolvePointers(left);
		SetLattice<DataframeOperation> df2 = resolvePointers(right);
		if (topOrBottom(df1) || topOrBottom(df2))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());

		DataframeOperation concatNode = new Concat(pp.getLocation(), Concat.Axis.CONCAT_COLS);
		forest.addNode(concatNode);

		for (DataframeOperation op : df1) {
			if (op instanceof AccessOperation<?>) {
				op = toProjection(right, forest, operations, op);
			}
			forest.addEdge(new ConcatEdge(op, concatNode, 0));
		}

		for (DataframeOperation op : df2) {
			if (op instanceof AccessOperation<?>)
				op = toProjection(right, forest, operations, op);

			forest.addEdge(new ConcatEdge(op, concatNode, 1));
		}

		// safety measure: remove edges if they exist
		DataframeEdge edge;
		for (DataframeOperation op1 : df1)
			for (DataframeOperation op2 : df2)
				if ((edge = forest.getEdgeConnecting(op1, op2)) != null)
					forest.getNodeList().removeEdge(edge);

		NodeId id = new NodeId(concatNode);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(ids),
				ops.putState(id, new SetLattice<>(concatNode)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doDropColumns(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(left);
		ConstantPropagation cols = right.constStack;
		if (topOrBottom(ops) || topOrBottom(cols) || !cols.is(List.class))
			return cleanStack(right, pp);

		List<ConstantPropagation> cs = cols.as(List.class);
		Set<String> accessedCols = new HashSet<>();
		ColumnListSelection colsSelection = null;
		for (ConstantPropagation c : cs) {
			if (topOrBottom(c) || !c.is(String.class)) {
				colsSelection = new ColumnListSelection(true);
				break;
			}
			accessedCols.add(c.as(String.class));
		}
		if (colsSelection == null)
			colsSelection = new ColumnListSelection(accessedCols);

		DropColumns drop = new DropColumns(pp.getLocation(), colsSelection);
		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(drop);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, drop));

		NodeId id = new NodeId(drop);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(ids),
				right.operations.putState(id, new SetLattice<>(drop)));
	}

	private static DataframeGraphDomain doFillNull(DataframeGraphDomain left,
			DataframeGraphDomain right, BinaryOperator operator, ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(left);
		ConstantPropagation value = right.constStack;
		if (topOrBottom(ops))
			return cleanStack(right, pp);

		FillNullAxis filler = new FillNullAxis(pp.getLocation(), ((FillNull) operator).getAxis(), value);

		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(filler);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, filler));

		NodeId id = new NodeId(filler);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(ids),
				right.operations.putState(id, new SetLattice<>(filler)));
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doColumnAccess(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(left);
		SetLattice<DataframeOperation> args = resolvePointers(right);
		ConstantPropagation col = right.constStack;
		if (topOrBottom(ops))
			return cleanStack(right, pp);

		Selection<?> selection = null;
		if (!topOrBottom(args)) {
			for (DataframeOperation arg : args)
				if (arg instanceof Iteration)
					selection = selection == null ? new ColumnIteration() : new ColumnListSelection(true);
				else if (arg instanceof BooleanComparison<?>)
					selection = selection == null ? new RowFilter(((BooleanComparison<?>) arg).getSelection())
							: new ColumnListSelection(true);
				else
					return cleanStack(right, pp);
		} else if (topOrBottom(col))
			selection = new ColumnListSelection(true);
		else if (col.is(String.class))
			selection = new ColumnListSelection(new Names(col.as(String.class)));
		else if (col.is(Integer.class))
			selection = new ColumnRangeSelection(col.as(Integer.class));
		else if (col.is(NumberSlice.class))
			selection = new ColumnRangeSelection(col.as(NumberSlice.class));
		else if (col.is(SliceConstant.class)) {
			SliceConstant c = col.as(SliceConstant.class);
			Slice slice = (Slice) c.getValue();
			selection = new ColumnRangeSelection(new NumberSlice(
					slice.getStart().toConstant(),
					slice.getEnd().toConstant(),
					slice.getSkip().toConstant()));
		} else if (col.is(List.class)) {
			List<ConstantPropagation> cs = col.as(List.class);
			Set<String> accessedCols = new HashSet<>();
			for (ConstantPropagation c : cs) {
				if (topOrBottom(c) || !c.is(String.class)) {
					selection = new ColumnListSelection(true);
					break;
				}
				accessedCols.add(c.as(String.class));
			}
			selection = new ColumnListSelection(accessedCols);
		} else
			selection = new ColumnListSelection(true);

		AccessOperation access = new AccessOperation(pp.getLocation(), selection);
		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(access);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, access));
		if (!topOrBottom(args))
			for (DataframeOperation arg : args)
				forest.addEdge(new ConsumeEdge(arg, access));

		NodeId id = new NodeId(access);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(ids),
				right.operations.putState(id, new SetLattice<>(access)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doListAppend(DataframeGraphDomain left, DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		ConstantPropagation list = left.constStack;
		if (topOrBottom(list) || topOrBottom(right) || !list.is(List.class))
			return right;

		Lattice<?> tail;
		if (!right.constStack.isBottom())
			tail = right.constStack;
		else if (!right.pointers.lattice.isBottom())
			tail = right.pointers.lattice;
		else
			tail = right.constants.lattice.top();

		ListConstant listconst = new ListConstant(pp.getLocation(), list.as(List.class), tail);
		return new DataframeGraphDomain(
				right.constants,
				right.constStack.eval(listconst, right.constants, pp),
				right.graph,
				right.pointers.setStack(NO_IDS),
				right.operations);
	}

	public DataframeGraphDomain visit(TernaryExpression expression, DataframeGraphDomain left,
			DataframeGraphDomain middle, DataframeGraphDomain right, ProgramPoint pp)
			throws SemanticException {
		if (left.isBottom())
			return left;
		if (middle.isBottom())
			return middle;
		if (right.isBottom())
			return right;

		TernaryOperator operator = expression.getOperator();

		if (operator == DictPut.INSTANCE)
			return doDictPut(left, middle, right, pp);
		else if (operator == ProjectRows.INSTANCE || operator == AccessRows.INSTANCE)
			return doAccessOrProjectRows(left, middle, right, pp, operator);
		else if (operator instanceof AccessRowsColumns)
			return doAccessRowsColumns(left, middle, right, pp);
		else if (operator instanceof SliceCreation)
			return doSliceCreation(left, middle, right, pp);
		else if (!left.constStack.isBottom()
				&& !middle.constStack.isBottom()
				&& !right.constStack.isBottom()
				&& right.constants.lattice.canProcess(expression))
			return delegateToConstants(expression, right, pp);
		else
			return cleanStack(right, pp);
	}

	private static DataframeGraphDomain doSliceCreation(DataframeGraphDomain left, DataframeGraphDomain middle,
			DataframeGraphDomain right, ProgramPoint pp) throws SemanticException {
		if (right.constStack.isBottom())
			return cleanStack(right, pp);
		RangeBound skip = getRangeBound(right.constStack);
		RangeBound rstart = null, rend = null;
		ColumnRangeSelection cstart = null, cend = null;
		SetLattice<DataframeOperation> l = null, m = null;

		if (!left.constStack.isBottom())
			rstart = getRangeBound(left.constStack);
		else if (!topOrBottom(left.pointers.lattice)) {
			l = resolvePointers(left);
			cstart = getRangeBound(l);
		}
		
		if (!middle.constStack.isBottom()) 
			rend = getRangeBound(middle.constStack);
		else if (!topOrBottom(middle.pointers.lattice)) {
			// column slice
			m = resolvePointers(middle);
			if (l.equals(m))
				l = moveBackwards(right.graph, l);
			cend = getRangeBound(m);
		}
		
		Constant slice;
		if (rstart != null && rend != null)
			slice = new SliceConstant(rstart, rend, skip, pp.getLocation());
		else
			slice = new DataframeColumnSlice(new ColumnSlice(
					rstart == null ? cstart : rstart, 
					rend == null ? cend : rend, skip, l, m), pp.getLocation());
		return new DataframeGraphDomain(
				right.constants,
				right.constStack.eval(slice, right.constants, pp),
				right.graph,
				right.pointers.setStack(NO_IDS),
				right.operations);
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doAccessRowsColumns(DataframeGraphDomain left, DataframeGraphDomain middle,
			DataframeGraphDomain right, ProgramPoint pp) throws SemanticException {
		// left[middle, right]
		// df[row_slice | column_comparison, columns]
		SetLattice<DataframeOperation> df = resolvePointers(left);
		// right is a list of strings so we will handle that first
		ConstantPropagation cols = right.constStack;
		if (topOrBottom(df) || topOrBottom(middle) || topOrBottom(cols))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);

		ColumnSelection<?> colsSelection = null;
		Set<DataframeOperation> toConsume = new HashSet<>();
		if (cols.is(List.class)) {
			List<Lattice<?>> cs = cols.as(List.class);
			Set<String> accessedCols = new HashSet<>();
			ConstantPropagation cc;
			for (Lattice<?> c : cs) {
				if (!(c instanceof ConstantPropagation) || topOrBottom(cc = (ConstantPropagation) c)
						|| !cc.is(String.class)) {
					colsSelection = new ColumnListSelection(true);
					break;
				}
				accessedCols.add(cc.as(String.class));
			}
			if (colsSelection == null)
				colsSelection = new ColumnListSelection(accessedCols);
		} else if (cols.is(ColumnSlice.class)) {
			ColumnSlice slice = cols.as(ColumnSlice.class);
			colsSelection = new ColumnRangeSelection(new NumberSlice(
					slice.getStart().getBeginIndex(),
					slice.getEnd().getEndIndex(),
					slice.getSkip().toConstant()));
			if (slice.getStartNodes() != null)
				// might be a numeric slice
				toConsume.addAll(slice.getStartNodes().elements());
			if (slice.getEndNodes() != null)
				// might be a numeric slice
				toConsume.addAll(slice.getEndNodes().elements());
		} else
			return cleanStack(right, pp);

		// middle can be either a slice or a series comparison
		if (!topOrBottom(middle.constStack)) {
			// middle is a slice
			SliceConstant.Slice rowSlice = middle.constStack.as(SliceConstant.Slice.class);
			NumberSlice numberSlice = new NumberSlice(
					rowSlice.getStart() == null ? new ConstantPropagation().bottom() : rowSlice.getStart().toConstant(),
					rowSlice.getEnd() == null ? new ConstantPropagation().bottom() : rowSlice.getEnd().toConstant(),
					rowSlice.getSkip() == null ? new ConstantPropagation().bottom() : rowSlice.getSkip().toConstant());
			DataframeSelection<?, ?> selection = new DataframeSelection(numberSlice, colsSelection);
			DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);
			forest.addNode(access);
			for (DataframeOperation op : df)
				forest.addEdge(new SimpleEdge(op, access));

			for (DataframeOperation op : toConsume)
				forest.addEdge(new ConsumeEdge(op, access));

			NodeId id = new NodeId(access);
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier,
					NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants,
					forest,
					pointers.setStack(ids),
					right.operations.putState(id, new SetLattice<>(access)));
		} else if (!topOrBottom(middle.pointers.lattice)) {
			SetLattice<DataframeOperation> middleGraph = resolvePointers(middle);
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
			Map<NodeId, DataframeOperation> ids = new HashMap<>();

			for (DataframeOperation op : middleGraph) {
				if (!(op instanceof BooleanComparison))
					return cleanStack(right, pp);

				BooleanComparison<?> colCompare = (BooleanComparison<?>) op;
				DataframeSelection<?,
						?> selection = new DataframeSelection(colCompare.getSelection(), colsSelection);
				DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);

				forest.addNode(access);
				forest.addEdge(new ConsumeEdge(op, access));

				NodeId id = new NodeId(access);
				ids.put(id, access);

				for (DataframeOperation opc : toConsume)
					forest.addEdge(new ConsumeEdge(opc, access));
			}

			SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
			for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
				ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
			CollectingMapLattice<Identifier,
					NodeId> pointers = shift(right.pointers, left.pointers.lattice, idsLattice);
			return new DataframeGraphDomain(
					right.constants,
					forest,
					pointers.setStack(idsLattice),
					ops);
		} else {
			DataframeSelection<?, ?> selection = new DataframeSelection<>(true);
			DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);
			forest.addNode(access);
			for (DataframeOperation op : df)
				forest.addEdge(new SimpleEdge(op, access));
			for (DataframeOperation op : toConsume)
				forest.addEdge(new ConsumeEdge(op, access));

			NodeId id = new NodeId(access);
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier,
					NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants,
					forest,
					pointers.setStack(ids),
					right.operations.putState(id, new SetLattice<>(access)));
		}
	}

	private static DataframeGraphDomain doAccessOrProjectRows(DataframeGraphDomain left, DataframeGraphDomain middle,
			DataframeGraphDomain right, ProgramPoint pp, TernaryOperator operator) throws SemanticException {
		SetLattice<DataframeOperation> df = resolvePointers(left);
		ConstantPropagation start = middle.constStack;
		ConstantPropagation end = right.constStack;
		if (topOrBottom(start) || topOrBottom(end) || topOrBottom(df))
			return cleanStack(right, pp);

		NumberSlice slice = new NumberSlice(start.as(Integer.class), end.as(Integer.class));
		DataframeOperation node;
		if (operator == ProjectRows.INSTANCE)
			node = new ProjectionOperation<>(pp.getLocation(), slice);
		else
			node = new AccessOperation<>(pp.getLocation(), slice);

		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(node);
		for (DataframeOperation op : df)
			forest.addEdge(new SimpleEdge(op, node));
		NodeId id = new NodeId(node);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers.setStack(ids),
				right.operations.putState(id, new SetLattice<>(node)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doDictPut(DataframeGraphDomain left, DataframeGraphDomain middle,
			DataframeGraphDomain right, ProgramPoint pp) throws SemanticException {
		ConstantPropagation dict = left.constStack;
		if (topOrBottom(dict) || topOrBottom(middle) || topOrBottom(right) || !dict.is(Map.class))
			return cleanStack(right, pp);

		Lattice<?> key, value;
		if (!middle.constStack.isBottom())
			key = middle.constStack;
		else if (!middle.pointers.lattice.isBottom())
			key = middle.pointers.lattice;
		else
			key = middle.constants.lattice.top();
		if (!right.constStack.isBottom())
			value = right.constStack;
		else if (!right.pointers.lattice.isBottom())
			value = right.pointers.lattice;
		else
			value = right.constants.lattice.top();

		DictConstant newdict = new DictConstant(pp.getLocation(), dict.as(Map.class), Pair.of(key, value));
		return new DataframeGraphDomain(
				right.constants,
				right.constStack.eval(newdict, right.constants, pp),
				right.graph,
				right.pointers.setStack(NO_IDS),
				right.operations);
	}

	private static DataframeGraphDomain cleanStack(DataframeGraphDomain right, ProgramPoint pp)
			throws SemanticException {
		LOG.debug("Evaluation of " + pp + " in " + getCaller() + " caused the stack to be cleaned");
		return new DataframeGraphDomain(
				right.constants,
				right.graph,
				right.pointers.setStack(NO_IDS),
				right.operations);
	}
	
	private static String getCaller() {
		StackTraceElement[] trace = Thread.getAllStackTraces().get(Thread.currentThread());
		// 0: java.lang.Thread.dumpThreads()
		// 1: java.lang.Thread.getAllStackTraces()
		// 2: DataframeGraphDomain.getCaller()
		// 3: DataframeGraphDomain.cleanStack()
		// 4: caller
		return trace[4].getClassName() + "::" + trace[4].getMethodName();
	}

	public DataframeGraphDomain visit(PushAny expression, ProgramPoint pp) throws SemanticException {
		if (isDataframeRelated(expression))
			return new DataframeGraphDomain(
					constants,
					graph,
					pointers.setStack(NO_IDS.top()),
					operations);
		else
			return new DataframeGraphDomain(
					constants,
					constStack.top(),
					graph,
					pointers.setStack(NO_IDS),
					operations);
	}

	public DataframeGraphDomain visit(Constant expression, ProgramPoint pp) throws SemanticException {
		return new DataframeGraphDomain(
				constants,
				constStack.eval(expression, constants, pp),
				graph,
				pointers.setStack(NO_IDS),
				operations);
	}

	public DataframeGraphDomain visit(Identifier expression, ProgramPoint pp) throws SemanticException {
		if (expression instanceof MemoryPointer)
			expression = ((MemoryPointer) expression).getReferencedLocation();

		if (expression instanceof AllocationSite)
			expression = stripFields((AllocationSite) expression);

		if (constants.getKeys().contains(expression))
			return new DataframeGraphDomain(
					constants,
					constants.getState(expression),
					graph,
					pointers.setStack(NO_IDS),
					operations);
		else if (pointers.getKeys().contains(expression))
			return new DataframeGraphDomain(
					constants,
					graph,
					pointers.setStack(pointers.getState(expression)),
					operations);
		else
			return new DataframeGraphDomain(
					constants,
					graph,
					pointers.setStack(NO_IDS),
					operations);
	}

	public static AllocationSite stripFields(AllocationSite as) {
		if (as.getField() != null)
			// we remove the name of the field using only location name
			if (as instanceof HeapAllocationSite)
				as = new HeapAllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(),
						as.getCodeLocation());
			else
				as = new StackAllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(),
						as.getCodeLocation());
		return as;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((constants == null) ? 0 : constants.hashCode());
		result = prime * result + ((constStack == null) ? 0 : constStack.hashCode());
		result = prime * result + ((graph == null) ? 0 : graph.hashCode());
		result = prime * result + ((operations == null) ? 0 : operations.hashCode());
		result = prime * result + ((pointers == null) ? 0 : pointers.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DataframeGraphDomain other = (DataframeGraphDomain) obj;
		if (constants == null) {
			if (other.constants != null)
				return false;
		} else if (!constants.equals(other.constants))
			return false;
		if (constStack == null) {
			if (other.constStack != null)
				return false;
		} else if (!constStack.equals(other.constStack))
			return false;
		if (graph == null) {
			if (other.graph != null)
				return false;
		} else if (!graph.equals(other.graph))
			return false;
		if (operations == null) {
			if (other.operations != null)
				return false;
		} else if (!operations.equals(other.operations))
			return false;
		if (pointers == null) {
			if (other.pointers != null)
				return false;
		} else if (!pointers.equals(other.pointers))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return representation().toString();
	}

	public DataframeForest close() {
		DataframeOperation close = new CloseOperation();

		DataframeForest result = new DataframeForest(graph);
		result.addNode(close);

		Collection<DataframeOperation> exits = new HashSet<>();
		for (SetLattice<NodeId> variable : pointers.getValues())
			for (NodeId id : variable)
				exits.addAll(operations.getState(id).elements());

		for (DataframeOperation op : exits)
			result.addEdge(new SimpleEdge(op, close));

		return result;
	}

	public static class CloseOperation extends DataframeOperation {

		public CloseOperation() {
			super(SyntheticLocation.INSTANCE);
		}

		@Override
		public String toString() {
			return "exit";
		}

		@Override
		protected DataframeOperation lubSameOperation(DataframeOperation other) throws SemanticException {
			return this;
		}

		@Override
		protected boolean lessOrEqualSameOperation(DataframeOperation other) throws SemanticException {
			return false;
		}

		@Override
		protected int compareToSameClassAndLocation(DataframeOperation o) {
			return 0;
		}
	}

	public Map<Identifier, DataframeForest> partitionByVarialbe() {
		Map<Identifier, DataframeForest> result = new HashMap<>();

		for (Entry<Identifier, SetLattice<NodeId>> entry : pointers.getMap().entrySet())
			if (!entry.getValue().isTop() && !entry.getValue().isBottom()) {
				DataframeForest accumulator = null;
				for (NodeId id : entry.getValue())
					for (DataframeOperation op : operations.getState(id)) {
						DataframeForest sub = graph.bDFS(op,
								o -> false,
								edge -> true);
						if (accumulator == null)
							accumulator = sub;
						else
							accumulator = accumulator.union(sub);
					}
				result.put(entry.getKey(), accumulator);
			}

		return result;
	}
}
