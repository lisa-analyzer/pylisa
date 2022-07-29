package it.unive.pylisa.analysis.dataframes.graph;

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

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.ObjectRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.Names;
import it.unive.pylisa.analysis.dataframes.transformation.graph.AssignEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.ConsumeEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AccessOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AssignDataframe;
import it.unive.pylisa.analysis.dataframes.transformation.operations.AssignValue;
import it.unive.pylisa.analysis.dataframes.transformation.operations.BooleanComparison;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Concat;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DropColumns;
import it.unive.pylisa.analysis.dataframes.transformation.operations.FilterNullAxis;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ProjectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.SelectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Transform;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.AtomicBooleanSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.ColumnListSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.transformation.operations.selection.NumberSlice;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyLibraryUnitType;
import it.unive.pylisa.symbolic.DictConstant;
import it.unive.pylisa.symbolic.ListConstant;
import it.unive.pylisa.symbolic.SliceConstant;
import it.unive.pylisa.symbolic.SliceConstant.RangeBound;
import it.unive.pylisa.symbolic.operators.DictPut;
import it.unive.pylisa.symbolic.operators.ListAppend;
import it.unive.pylisa.symbolic.operators.SliceCreation;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRows;
import it.unive.pylisa.symbolic.operators.dataframes.AccessRowsColumns;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation.Kind;
import it.unive.pylisa.symbolic.operators.dataframes.AxisConcatenation;
import it.unive.pylisa.symbolic.operators.dataframes.ColumnAccess;
import it.unive.pylisa.symbolic.operators.dataframes.CopyDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.DropCols;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull;
import it.unive.pylisa.symbolic.operators.dataframes.JoinCols;
import it.unive.pylisa.symbolic.operators.dataframes.PandasSeriesComparison;
import it.unive.pylisa.symbolic.operators.dataframes.PopSelection;
import it.unive.pylisa.symbolic.operators.dataframes.ProjectRows;
import it.unive.pylisa.symbolic.operators.dataframes.ReadDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionConstant;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionDataframe;

public class DataframeGraphDomain implements ValueDomain<DataframeGraphDomain> {

	private static final SetLattice<NodeId> NO_IDS = new SetLattice<NodeId>().bottom();
	private static final SetLattice<DataframeOperation> NO_NODES = new SetLattice<DataframeOperation>().bottom();

	private final ValueEnvironment<ConstantPropagation> constants;

	private final DataframeForest graph;

	private final CollectingMapLattice<Identifier, NodeId> pointers;

	private final CollectingMapLattice<NodeId, DataframeOperation> operations;

	public DataframeGraphDomain() {
		this.constants = new ValueEnvironment<>(new ConstantPropagation()).top();
		this.graph = new DataframeForest(true).top();
		this.pointers = new CollectingMapLattice<Identifier, NodeId>(new SetLattice<>()).top();
		this.operations = new CollectingMapLattice<NodeId, DataframeOperation>(new SetLattice<>()).top();
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
		super();
		this.constants = constants;
		this.graph = graph;
		this.pointers = pointers;

		// cleanup unreachable nodes
//		Map<NodeId, SetLattice<DataframeOperation>> map = operations.getMap();
//		if (map != null && !map.isEmpty()) {
//			Set<NodeId> nodes = new HashSet<>(operations.getKeys());
//			for (SetLattice<NodeId> used : this.pointers.getValues())
//				used.forEach(nodes::remove);
//			pointers.lattice.forEach(nodes::remove);
//			nodes.forEach(map::remove);
//			this.operations = new CollectingMapLattice<>(operations.lattice, map);
//		} else
			this.operations = operations;

		// FIXME temporary sanity check
		SetLattice<DataframeOperation> pointed = resolvePointers(this);
		for (DataframeOperation op : pointed)
			if (!graph.containsNode(op))
				throw new IllegalStateException();
	}

	@Override
	public DataframeGraphDomain assign(Identifier id, ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		DataframeGraphDomain sss = smallStepSemantics(expression, pp);
		if (!sss.constants.getValueOnStack().isBottom())
			return new DataframeGraphDomain(
					sss.constants.putState(id, sss.constants.getValueOnStack()),
					sss.graph,
					sss.pointers,
					sss.operations);
		else if (!sss.pointers.lattice.isBottom())
			return new DataframeGraphDomain(
					sss.constants,
					sss.graph,
					sss.pointers.putState(id, sss.pointers.lattice),
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
	public DataframeGraphDomain assume(ValueExpression expression, ProgramPoint pp) throws SemanticException {
		return new DataframeGraphDomain(
				constants.assume(expression, pp),
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
		return new DataframeGraphDomain(
				constants.pushScope(token),
				graph,
				pointers.lift(id -> (Identifier) id.pushScope(token), e -> e),
				operations);
	}

	@Override
	public DataframeGraphDomain popScope(ScopeToken token) throws SemanticException {
		return new DataframeGraphDomain(
				constants.popScope(token),
				graph,
				pointers.lift(id -> (Identifier) id.popScope(token), e -> e),
				operations);
	}

	@Override
	public DomainRepresentation representation() {
		return new ObjectRepresentation(Map.of(
				"constants", constants.representation(),
				"pointers", pointers.representation(StringRepresentation::new),
				"operations", operations.representation(StringRepresentation::new),
				"graph", graph.representation()));
	}

	@Override
	public DataframeGraphDomain lub(DataframeGraphDomain other) throws SemanticException {
		return new DataframeGraphDomain(
				constants.lub(other.constants),
				graph.lub(other.graph),
				pointers.lub(other.pointers),
				operations.lub(other.operations));
	}

	@Override
	public DataframeGraphDomain widening(DataframeGraphDomain other) throws SemanticException {
		return new DataframeGraphDomain(
				constants.widening(other.constants),
				graph.widening(other.graph),
				pointers.widening(other.pointers),
				operations.widening(other.operations));
	}

	@Override
	public boolean lessOrEqual(DataframeGraphDomain other) throws SemanticException {
		return constants.lessOrEqual(other.constants)
				&& graph.lessOrEqual(other.graph)
				&& pointers.lessOrEqual(other.pointers)
				&& operations.lessOrEqual(other.operations);
	}

	@Override
	public DataframeGraphDomain top() {
		return new DataframeGraphDomain(
				constants.top(),
				graph.top(),
				pointers.top(),
				operations.top());
	}

	@Override
	public DataframeGraphDomain bottom() {
		return new DataframeGraphDomain(
				constants.bottom(),
				graph.bottom(),
				pointers.bottom(),
				operations.bottom());
	}

	public static boolean isDataframeRelated(SymbolicExpression expression) {
		return expression.hasRuntimeTypes()
				? expression.getRuntimeTypes()
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

	private static DataframeOperation toProjection(
			DataframeGraphDomain domain,
			DataframeForest forest,
			Map<NodeId, SetLattice<DataframeOperation>> finalOperations,
			DataframeOperation access) {
		DataframeOperation tmp = new ProjectionOperation<>(access.getWhere(),
				((AccessOperation<?>) access).getSelection());
		forest.replace(access, tmp);
		for (Entry<NodeId, SetLattice<DataframeOperation>> entry : domain.operations)
			if (entry.getValue().contains(access))
				finalOperations.put(entry.getKey(), finalOperations.get(entry.getKey()).replace(access, tmp));
		return tmp;
	}

	@SuppressWarnings({ "rawtypes", "unchecked" })
	public DataframeGraphDomain visit(UnaryExpression expression, DataframeGraphDomain arg, ProgramPoint pp)
			throws SemanticException {
		if (arg.isBottom())
			return arg;

		UnaryOperator operator = expression.getOperator();
		if (operator == ReadDataframe.INSTANCE) {
			ConstantPropagation filename = arg.constants.getValueOnStack();
			if (topOrBottom(filename))
				return arg.top();

			DataframeForest df = new DataframeForest(arg.graph);
			ReadFromFile op = new ReadFromFile(pp.getLocation(), filename.as(String.class));
			df.addNode(op);

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			SetLattice<DataframeOperation> operations = new SetLattice<>(op);
			// no shift necessary: this is a new dataframe creation
			return new DataframeGraphDomain(
					arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					df,
					arg.pointers.setStack(ids),
					arg.operations.putState(id, operations));
		} else if (operator == CopyDataframe.INSTANCE) {
			if (topOrBottom(arg.pointers.lattice))
				return arg.top();

			Set<NodeId> ids = new HashSet<>();
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.operations.getMap());
			for (NodeId id : arg.pointers.lattice) {
				NodeId copy = new NodeId();
				ids.add(copy);
				operations.put(copy, arg.operations.getState(id));
			}
			// no shift necessary: this is a new dataframe creation
			return new DataframeGraphDomain(
					arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					arg.graph,
					arg.pointers.setStack(new SetLattice<>(ids, false)),
					new CollectingMapLattice<>(arg.operations.lattice, operations));
		} else if (operator instanceof ApplyTransformation) {
			SetLattice<DataframeOperation> stack = resolvePointers(arg);
			if (topOrBottom(stack))
				return arg.top();

			ApplyTransformation op = (ApplyTransformation) operator;
			Kind kind = op.getKind();

			DataframeForest df = new DataframeForest(arg.graph);
			Set<DataframeOperation> newStack = new HashSet<>();
			Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(arg.operations.getMap());
			for (DataframeOperation leaf : stack) {
				if (!(leaf instanceof SelectionOperation<?>))
					return arg.top();
				
				Transform t = op.getArg().isPresent()
						? new Transform(pp.getLocation(), kind, ((SelectionOperation<?>) leaf).getSelection(),
								op.getArg().get())
						: new Transform(pp.getLocation(), kind, ((SelectionOperation<?>) leaf).getSelection());

				df.addNode(t);
				df.addEdge(new ConsumeEdge(leaf, t));
				newStack.add(t);
			}

			SetLattice<DataframeOperation> ops = new SetLattice<>(newStack, false);
			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, ids);
			return new DataframeGraphDomain(
					arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					df,
					pointers.setStack(ids),
					new CollectingMapLattice<>(arg.operations.lattice, map).putState(id, ops));
		} else if (operator instanceof FilterNull) {
			SetLattice<DataframeOperation> ops = resolvePointers(arg);
			if (topOrBottom(ops))
				return arg.top();

			FilterNullAxis filter = new FilterNullAxis(pp.getLocation(), ((FilterNull) operator).getAxis());

			DataframeForest forest = new DataframeForest(arg.graph);
			forest.addNode(filter);
			for (DataframeOperation op : ops)
				forest.addEdge(new SimpleEdge(op, filter));

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.pointers, arg.pointers.lattice, ids);
			return new DataframeGraphDomain(
					arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					arg.operations.putState(id, new SetLattice<>(filter)));
		} else if (operator instanceof PopSelection) {
			SetLattice<DataframeOperation> ops = resolvePointers(arg);
			if (topOrBottom(ops))
				return arg.top();

			DataframeForest forest = new DataframeForest(arg.graph);
			Set<NodeId> ids = new HashSet<>();
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.operations.getMap());
			for (DataframeOperation op : ops) {
				if (!(op instanceof SelectionOperation<?>))
					return arg.top();

				Collection<DataframeOperation> preds = forest.predecessorsOf(op);
				if (preds.size() != 1)
					throw new SemanticException("Not supported yet");
				DataframeOperation pred = preds.iterator().next();
				forest.getAdjacencyMatrix().removeNode(op);
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
					arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(idsLattice),
					new CollectingMapLattice<>(arg.operations.lattice, operations));
		} else if (operator instanceof AxisConcatenation) {
			ConstantPropagation list = arg.constants.getValueOnStack();
			if (topOrBottom(list) || !list.is(List.class))
				return arg.top();

			List<Lattice<?>> elements = list.as(List.class);
			if (elements.isEmpty())
				return arg.top();

			Lattice<?> firstWrapped = elements.iterator().next();
			if (elements.size() == 1)
				if (firstWrapped instanceof SetLattice<?>)
				return new DataframeGraphDomain(
						arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						arg.graph,
						arg.pointers.setStack((SetLattice<NodeId>) firstWrapped),
						arg.operations);
				else 
					return arg.top();

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
					return arg.top();

				SetLattice<DataframeOperation> ops = resolvePointers(arg.operations, (SetLattice<NodeId>) element);
				if (entry.getValue().size() == 1) 
					// appears once
					operands[entry.getValue().iterator().next()] = ops;
				else {
					// appears more than once
					Set<DataframeOperation> operand = ops.elements();
					FIFOWorkingSet<SetLattice<DataframeOperation>> ws = FIFOWorkingSet.mk();
					for (int i = 0; i < entry.getValue().size(); i++) {
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
						operand = fixed.stream().flatMap(o -> forest.predecessorsOf(o).stream()).distinct().collect(Collectors.toSet());
					}

					for (int pos : entry.getValue())
						operands[pos] = ws.pop();
				}
			}

			for (int i = 0; i < operands.length; i++) {
				SetLattice<DataframeOperation> ops = operands[i];
				for (DataframeOperation exit : ops)
					forest.addEdge(new ConcatEdge(exit, concatNode, i));
			}

			NodeId id = new NodeId();
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(arg.operations.lattice, operations);
			// no shift necessary: this is a new dataframe creation
			return new DataframeGraphDomain(
					arg.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					arg.pointers.setStack(new SetLattice<>(id)),
					ops.putState(id, new SetLattice<>(concatNode)));
		} else
			return arg.top();
	}

	@SuppressWarnings("unchecked")
	public DataframeGraphDomain visit(BinaryExpression expression, DataframeGraphDomain left,
			DataframeGraphDomain right, ProgramPoint pp) throws SemanticException {
		if (left.isBottom())
			return left;
		if (right.isBottom())
			return right;

		BinaryOperator operator = expression.getOperator();

		if (operator == ListAppend.INSTANCE) {
			ConstantPropagation list = left.constants.getValueOnStack();
			if (topOrBottom(list) || topOrBottom(right) || !list.is(List.class))
				return right;

			Lattice<?> tail;
			if (!right.constants.getValueOnStack().isBottom())
				tail = right.constants.getValueOnStack();
			else if (!right.pointers.lattice.isBottom())
				tail = right.pointers.lattice;
			else
				tail = right.constants.lattice.top();

			ListConstant listconst = new ListConstant(pp.getLocation(), list.as(List.class), tail);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(listconst, pp),
					right.graph,
					right.pointers.setStack(NO_IDS),
					right.operations);
		} else if (operator == ColumnAccess.INSTANCE) {
			SetLattice<DataframeOperation> ops = resolvePointers(left);
			ConstantPropagation col = right.constants.getValueOnStack();
			if (topOrBottom(ops) || topOrBottom(col))
				return right.top();

			ColumnListSelection columns;
			if (col.is(String.class))
				columns = new ColumnListSelection(new Names(col.as(String.class)));
			else {
				List<ConstantPropagation> cs = col.as(List.class);
				Set<String> accessedCols = new HashSet<>();
				for (ConstantPropagation c : cs) {
					if (topOrBottom(c) || !c.is(String.class)) {
						columns = new ColumnListSelection(true);
						break;
					}
					accessedCols.add(c.as(String.class));
				}
				columns = new ColumnListSelection(accessedCols);
			}

			AccessOperation<ColumnListSelection> access = new AccessOperation<>(pp.getLocation(), columns);
			DataframeForest forest = new DataframeForest(right.graph);
			forest.addNode(access);
			for (DataframeOperation op : ops)
				forest.addEdge(new SimpleEdge(op, access));

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					right.operations.putState(id, new SetLattice<>(access)));
		} else if (operator == DropCols.INSTANCE) {
			SetLattice<DataframeOperation> ops = resolvePointers(left);
			ConstantPropagation cols = right.constants.getValueOnStack();
			if (topOrBottom(ops) || topOrBottom(cols) || !cols.is(List.class))
				return right.top();

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

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					right.operations.putState(id, new SetLattice<>(drop)));
		} else if (operator == JoinCols.INSTANCE) {
			SetLattice<DataframeOperation> df1 = resolvePointers(left);
			SetLattice<DataframeOperation> df2 = resolvePointers(right);
			if (topOrBottom(df1) || topOrBottom(df2))
				return right.top();

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
						forest.getAdjacencyMatrix().removeEdge(edge);

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					ops.putState(id, new SetLattice<>(concatNode)));
		} else if (operator instanceof WriteSelectionDataframe) {
			SetLattice<DataframeOperation> df1 = resolvePointers(left);
			SetLattice<DataframeOperation> df2 = resolvePointers(right);
			if (topOrBottom(df1) || topOrBottom(df2))
				return right.top();

			DataframeForest forest = new DataframeForest(right.graph);
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
			Set<DataframeOperation> assignments = new HashSet<>();

			for (DataframeOperation op : df1) {
				if (!(op instanceof SelectionOperation<?>))
					return right.top();

				AssignDataframe<?> assign = new AssignDataframe<>(pp.getLocation(),
						((SelectionOperation<?>) op).getSelection());
				forest.addNode(assign);
				forest.addEdge(new ConsumeEdge(op, assign));
				assignments.add(assign);
			}

			for (DataframeOperation op : df2) {
				if (op instanceof AccessOperation<?>) 
					op = toProjection(right, forest, operations, op);

				for (DataframeOperation assign : assignments)
					forest.addEdge(new AssignEdge(op, assign));
			}

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					ops.putState(id, new SetLattice<>(assignments, false)));
		} else if (operator instanceof WriteSelectionConstant) {
			SetLattice<DataframeOperation> df = resolvePointers(left);
			ConstantPropagation c = right.constants.getValueOnStack();
			if (topOrBottom(df) || topOrBottom(c))
				return right.top();

			DataframeForest forest = new DataframeForest(right.graph);
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
			Set<DataframeOperation> assignments = new HashSet<>();

			for (DataframeOperation op : df) {
				if (!(op instanceof SelectionOperation))
					return right.top();

				SelectionOperation<?> access = (SelectionOperation<?>) op;
				DataframeSelection<?, ?> selection = (DataframeSelection<?, ?>) access.getSelection();
				DataframeOperation nodeToAdd = new AssignValue<>(pp.getLocation(), selection, c);
				forest.addNode(nodeToAdd);
				forest.addEdge(new ConsumeEdge(op, nodeToAdd));
				assignments.add(nodeToAdd);
			}

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					ops.putState(id, new SetLattice<>(assignments, false)));
		} else if (operator instanceof PandasSeriesComparison) {
			SetLattice<DataframeOperation> df1 = resolvePointers(left);
			ConstantPropagation value = right.constants.getValueOnStack();
			if (topOrBottom(df1) || topOrBottom(value))
				return right.top();

			DataframeForest forest = new DataframeForest(right.graph);
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
			Set<DataframeOperation> comparisons = new HashSet<>();
			PandasSeriesComparison seriesCompOp = (PandasSeriesComparison) operator;

			for (DataframeOperation op : df1) {
				if (!(op instanceof SelectionOperation))
					return right.top();

				SelectionOperation<?> projection = (SelectionOperation<?>) op;
				if (!(projection.getSelection() instanceof ColumnListSelection))
					return right.top();

				AtomicBooleanSelection booleanSelection = new AtomicBooleanSelection(
						(ColumnListSelection) projection.getSelection(), seriesCompOp.getOp(), value);
				BooleanComparison<
						AtomicBooleanSelection> boolComp = new BooleanComparison<>(pp.getLocation(),
								booleanSelection);

				forest.addNode(boolComp);
				forest.addEdge(new ConsumeEdge(op, boolComp));
				comparisons.add(boolComp);
			}

			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
//			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					ops.putState(id, new SetLattice<>(comparisons, false)));
		} else
			return DataframeGraphDomain.this;
	}

	@SuppressWarnings("unchecked")
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

		if (operator == DictPut.INSTANCE) {
			ConstantPropagation dict = left.constants.getValueOnStack();
			if (topOrBottom(dict) || topOrBottom(middle) || topOrBottom(right) || !dict.is(Map.class))
				return right.top();

			Lattice<?> key, value;
			if (!middle.constants.getValueOnStack().isBottom())
				key = middle.constants.getValueOnStack();
			else if (!middle.pointers.lattice.isBottom())
				key = middle.pointers.lattice;
			else
				key = middle.constants.lattice.top();
			if (!right.constants.getValueOnStack().isBottom())
				value = right.constants.getValueOnStack();
			else if (!right.pointers.lattice.isBottom())
				value = right.pointers.lattice;
			else
				value = right.constants.lattice.top();

			DictConstant newdict = new DictConstant(pp.getLocation(), dict.as(Map.class), Pair.of(key, value));
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(newdict, pp),
					right.graph,
					right.pointers.setStack(NO_IDS),
					right.operations);
		} else if (operator == ProjectRows.INSTANCE || operator == AccessRows.INSTANCE) {
			SetLattice<DataframeOperation> df = resolvePointers(left);
			ConstantPropagation start = middle.constants.getValueOnStack();
			ConstantPropagation end = right.constants.getValueOnStack();
			if (topOrBottom(start) || topOrBottom(end) || topOrBottom(df))
				return right.top();

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
			NodeId id = new NodeId();
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier, NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					forest,
					pointers.setStack(ids),
					right.operations.putState(id, new SetLattice<>(node)));
		} else if (operator instanceof AccessRowsColumns) {
			// left[middle, right]
			// df[row_slice | column_comparison, columns]
			SetLattice<DataframeOperation> df = resolvePointers(left);
			// right is a list of strings so we will handle that first
			ConstantPropagation cols = right.constants.getValueOnStack();
			if (topOrBottom(df) || topOrBottom(middle) || topOrBottom(cols) || !cols.is(List.class))
				return right.top();

			DataframeForest forest = new DataframeForest(right.graph);

			List<Lattice<?>> cs = cols.as(List.class);
			Set<String> accessedCols = new HashSet<>();
			ColumnListSelection colsSelection = null;
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

			// middle can be either a slice or a series comparison
			if (!topOrBottom(middle.constants.getValueOnStack())) {
				// middle is a slice
				SliceConstant.Slice rowSlice = middle.constants.getValueOnStack().as(SliceConstant.Slice.class);
				NumberSlice numberSlice = new NumberSlice(
						rowSlice.getStart() == null ? new Interval().bottom() : rowSlice.getStart().toInterval(),
						rowSlice.getEnd() == null ? new Interval().bottom() : rowSlice.getEnd().toInterval(),
						rowSlice.getSkip() == null ? new Interval().bottom() : rowSlice.getSkip().toInterval());
				DataframeSelection<?, ?> selection = new DataframeSelection<>(numberSlice, colsSelection);
				DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);
				forest.addNode(access);
				for (DataframeOperation op : df)
					forest.addEdge(new SimpleEdge(op, access));

				NodeId id = new NodeId();
				SetLattice<NodeId> ids = new SetLattice<>(id);
				CollectingMapLattice<Identifier,
						NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
				return new DataframeGraphDomain(
						right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						forest,
						pointers.setStack(ids),
						right.operations.putState(id, new SetLattice<>(access)));
			} else if (!topOrBottom(middle.pointers.lattice)) {
				SetLattice<DataframeOperation> middleGraph = resolvePointers(middle);
				Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.operations.getMap());
				Set<DataframeOperation> accesses = new HashSet<>();

				for (DataframeOperation op : middleGraph) {
					if (!(op instanceof BooleanComparison))
						return right.top();

					BooleanComparison<?> colCompare = (BooleanComparison<?>) op;
					DataframeSelection<?,
							?> selection = new DataframeSelection<>(colCompare.getSelection(), colsSelection);
					DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);

					forest.addNode(access);
					forest.addEdge(new ConsumeEdge(op, access));
					accesses.add(access);
				}

				NodeId id = new NodeId();
				SetLattice<NodeId> ids = new SetLattice<>(id);
				CollectingMapLattice<NodeId,
						DataframeOperation> ops = new CollectingMapLattice<>(right.operations.lattice, operations);
				CollectingMapLattice<Identifier,
						NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
				return new DataframeGraphDomain(
						right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						forest,
						pointers.setStack(ids),
						ops.putState(id, new SetLattice<>(accesses, false)));

			} else {
				DataframeSelection<?, ?> selection = new DataframeSelection<>(true);
				DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);
				forest.addNode(access);
				for (DataframeOperation op : df)
					forest.addEdge(new SimpleEdge(op, access));

				NodeId id = new NodeId();
				SetLattice<NodeId> ids = new SetLattice<>(id);
				CollectingMapLattice<Identifier,
						NodeId> pointers = shift(right.pointers, left.pointers.lattice, ids);
				return new DataframeGraphDomain(
						right.constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						forest,
						pointers.setStack(ids),
						right.operations.putState(id, new SetLattice<>(access)));
			}
		} else if (operator instanceof SliceCreation) {
			if (left.constants.getValueOnStack().isBottom()
					|| middle.constants.getValueOnStack().isBottom()
					|| right.constants.getValueOnStack().isBottom()) {
				return right.top();
			}
			RangeBound start = getRangeBound(left.constants.getValueOnStack());
			RangeBound end = getRangeBound(middle.constants.getValueOnStack());
			RangeBound skip = getRangeBound(right.constants.getValueOnStack());
			SliceConstant slice = new SliceConstant(start, end, skip, pp.getLocation());
			return new DataframeGraphDomain(
					right.constants.smallStepSemantics(slice, pp),
					right.graph,
					right.pointers.setStack(NO_IDS),
					right.operations);
		} else
			return DataframeGraphDomain.this;
	}

	public DataframeGraphDomain visit(PushAny expression, ProgramPoint pp) throws SemanticException {
		if (isDataframeRelated(expression))
			return new DataframeGraphDomain(
					constants.smallStepSemantics(expression, pp),
					graph,
					pointers.setStack(NO_IDS.top()),
					operations);
		else
			return new DataframeGraphDomain(
					constants.smallStepSemantics(expression, pp),
					graph,
					pointers.setStack(NO_IDS),
					operations);
	}

	public DataframeGraphDomain visit(Constant expression, ProgramPoint pp) throws SemanticException {
		return new DataframeGraphDomain(
				constants.smallStepSemantics(expression, pp),
				graph,
				pointers.setStack(NO_IDS),
				operations);
	}

	public DataframeGraphDomain visit(Identifier expression, ProgramPoint pp) throws SemanticException {
		if (expression instanceof MemoryPointer)
			expression = ((MemoryPointer) expression).getReferencedLocation();
		
		if (expression instanceof AllocationSite) {
			// TODO this is very fragile and only works with the current state
			// of the field sensitive program point based heap
			AllocationSite as = (AllocationSite) expression;
			if (as.getName().endsWith("]")) 
				// we remove the name of the field using only location name
				expression = new AllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(), as.getCodeLocation());
		}
		
		if (constants.getKeys().contains(expression))
			return new DataframeGraphDomain(
					constants.smallStepSemantics(expression, pp),
					graph,
					pointers.setStack(NO_IDS),
					operations);
		else if (pointers.getKeys().contains(expression))
			return new DataframeGraphDomain(
					constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					graph,
					pointers.setStack(pointers.getState(expression)),
					operations);
		else
			return new DataframeGraphDomain(
					constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
					graph,
					pointers.setStack(NO_IDS),
					operations);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((constants == null) ? 0 : constants.hashCode());
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
}
