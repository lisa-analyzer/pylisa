package it.unive.pylisa.analysis.dataframes.graph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Predicate;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.ObjectRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.ExpressionVisitor;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapAllocation;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Skip;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.SelectionOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Transform;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyLibraryUnitType;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation;
import it.unive.pylisa.symbolic.operators.dataframes.ApplyTransformation.Kind;
import it.unive.pylisa.symbolic.operators.dataframes.ReadDataframe;

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
		this.operations = operations;
	}
	
	@Override
	public DataframeGraphDomain assign(Identifier id, ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		DataframeGraphDomain sss = smallStepSemantics(expression, pp);
		if (!sss.constants.getValueOnStack().isBottom())
			return new DataframeGraphDomain(
					sss.constants.putState(id, constants.getValueOnStack()),
					sss.graph,
					sss.pointers,
					sss.operations);
		else if (!sss.pointers.lattice.isBottom())
			return new DataframeGraphDomain(
					sss.constants,
					sss.graph,
					sss.pointers.putState(id, pointers.lattice),
					sss.operations);
		else if (!sss.operations.lattice.isBottom()) {
			Set<NodeId> nodes = new HashSet<>();
			Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(sss.operations.getMap());

			for (DataframeOperation op : sss.operations.lattice) {
				NodeId node = new NodeId(op);
				nodes.add(node);
				map.put(node, new SetLattice<>(op));
			}

			CollectingMapLattice<NodeId,
					DataframeOperation> newOps = new CollectingMapLattice<>(sss.operations.lattice, map);
			SetLattice<NodeId> approx = new SetLattice<>(nodes, false);
			return new DataframeGraphDomain(
					sss.constants,
					sss.graph,
					sss.pointers.putState(id, approx).setStack(approx),
					newOps);

		} else
			return this;
	}

	@Override
	public DataframeGraphDomain smallStepSemantics(ValueExpression expression, ProgramPoint pp)
			throws SemanticException {
		return expression.accept(new EvaluationVisitor(), this, pp);
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

	private class EvaluationVisitor implements ExpressionVisitor<DataframeGraphDomain> {

		private static final String CANNOT_PROCESS_ERROR = "Cannot process a heap expression with a non-relational value domain";

		@Override
		public DataframeGraphDomain visit(AccessChild expression, DataframeGraphDomain receiver,
				DataframeGraphDomain child, Object... params) throws SemanticException {
			throw new SemanticException(CANNOT_PROCESS_ERROR);
		}

		@Override
		public DataframeGraphDomain visit(HeapAllocation expression, Object... params) throws SemanticException {
			throw new SemanticException(CANNOT_PROCESS_ERROR);
		}

		@Override
		public DataframeGraphDomain visit(HeapReference expression, DataframeGraphDomain arg, Object... params)
				throws SemanticException {
			throw new SemanticException(CANNOT_PROCESS_ERROR);
		}

		@Override
		public DataframeGraphDomain visit(HeapDereference expression, DataframeGraphDomain arg, Object... params)
				throws SemanticException {
			throw new SemanticException(CANNOT_PROCESS_ERROR);
		}

		@SuppressWarnings({ "rawtypes", "unchecked" })
		@Override
		public DataframeGraphDomain visit(UnaryExpression expression, DataframeGraphDomain arg, Object... params)
				throws SemanticException {
			if (arg.isBottom())
				return arg;

			ProgramPoint pp = (ProgramPoint) params[1];
			UnaryOperator operator = expression.getOperator();
			if (operator == ReadDataframe.INSTANCE) {
				ConstantPropagation filename = arg.constants.getValueOnStack();
				if (topOrBottom(filename))
					return arg.top();

				DataframeForest df = new DataframeForest(arg.graph);
				ReadFromFile op = new ReadFromFile(pp.getLocation(), filename.as(String.class));
				df.addNode(op);
				return new DataframeGraphDomain(
						constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						df,
						pointers.setStack(NO_IDS),
						operations.setStack(new SetLattice<>(op)));
			} else if (operator instanceof ApplyTransformation) {
				SetLattice<DataframeOperation> stack = arg.operations.getLattice();
				if (topOrBottom(stack))
					return arg.top();

				ApplyTransformation op = (ApplyTransformation) operator;
				Kind kind = op.getKind();

				DataframeForest df = new DataframeForest(arg.graph);
				Set<DataframeOperation> newStack = new HashSet<>();
//				Set<NodeId> toShift = new HashSet<>();
				for (DataframeOperation leaf : stack) {
					if (!(leaf instanceof SelectionOperation<?>))
						return arg.top();

					Transform t = op.getArg().isPresent()
							? new Transform(pp.getLocation(), kind, ((SelectionOperation<?>) leaf).getSelection(),
									op.getArg().get())
							: new Transform(pp.getLocation(), kind, ((SelectionOperation<?>) leaf).getSelection());

					df.addNode(t);
					newStack.add(t);
//					for (Entry<NodeId, SetLattice<DataframeOperation>> entry : operations)
//						if (entry.getValue().contains(leaf))
//							toShift.add(entry.getKey());
				}

//				Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(arg.operations.getMap());
				SetLattice<DataframeOperation> operations = new SetLattice<>(newStack, false);
//				for (NodeId node : toShift)
//					map.put(node, operations);
//				CollectingMapLattice<NodeId,
//						DataframeOperation> newOps = new CollectingMapLattice<>(arg.operations.lattice, map);
				return new DataframeGraphDomain(
						constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						df,
						pointers.setStack(NO_IDS),
						arg.operations.setStack(operations));
			} /*
				 * lse if (operator instanceof FilterNull) {
				 * DataframeGraphDomain df = arg.graph; if (topOrBottom(df))
				 * return arg.top(); DataframeGraphDomain dfNew = new
				 * DataframeGraphDomain(df, new FilterNullAxis(pp.getLocation(),
				 * ((FilterNull) operator).getAxis())); return new
				 * DFOrConstant(dfNew); } else if (operator instanceof
				 * PopSelection) { DataframeGraphDomain df = arg.graph; if
				 * (topOrBottom(df)) return arg.top(); DataframeOperation leaf =
				 * df.getTransformations().getLeaf(); if (!(leaf instanceof
				 * SelectionOperation<?>)) return arg.top();
				 * DataframeGraphDomain popped = new
				 * DataframeGraphDomain(df.getTransformations().prefix());
				 * return new DFOrConstant(popped); } else if (operator
				 * instanceof AxisConcatenation) { ConstantPropagation list =
				 * arg.constant; if (topOrBottom(list) || !list.is(List.class))
				 * return arg.top(); List<Lattice<?>> elements =
				 * list.as(List.class); if (elements.isEmpty()) return
				 * arg.bottom(); DFOrConstant firstWrapped = (DFOrConstant)
				 * elements.iterator().next(); if (elements.size() == 1) return
				 * firstWrapped; DataframeGraph concatGraph = new
				 * DataframeGraph(); DataframeOperation concatNode = new
				 * Concat(pp.getLocation(), operator == JoinCols.INSTANCE ?
				 * Concat.Axis.CONCAT_COLS : Concat.Axis.CONCAT_ROWS);
				 * concatGraph.addNode(concatNode); // 1) if a leaf is an access
				 * it has to become a projection // 2) the same graph might
				 * appear multiple times as different nodes // of the concat int
				 * nelements = elements.size(); List<Lattice<?>>
				 * distinctElements =
				 * elements.stream().distinct().collect(Collectors.toList());
				 * int[] opIndexes = new int[nelements]; Map<Integer,
				 * List<Integer>> positions = new HashMap<>(); for (int i = 0; i
				 * < nelements; i++) { int idx =
				 * distinctElements.indexOf(elements.get(i)); opIndexes[i] =
				 * idx; positions.computeIfAbsent(idx, index -> new
				 * LinkedList<>()).add(i); } Map<Integer,
				 * LIFOWorkingSet<DataframeOperation>> leaves = new HashMap<>();
				 * DataframeGraph[] operands = new DataframeGraph[nelements];
				 * for (Entry<Integer, List<Integer>> entry :
				 * positions.entrySet()) { DataframeGraph original =
				 * ((DFOrConstant) distinctElements.get(entry.getKey())).graph
				 * .getTransformations(); LIFOWorkingSet<DataframeOperation>
				 * leavesWs = LIFOWorkingSet.mk(); if (entry.getValue().size()
				 * == 1) { // appears once
				 * operands[entry.getValue().iterator().next()] = original;
				 * leavesWs.push(original.getLeaf()); leaves.put(entry.getKey(),
				 * leavesWs); } else { // appears more than once DataframeGraph
				 * operand = original; FIFOWorkingSet<DataframeOperation> ws =
				 * FIFOWorkingSet.mk(); for (int i = 0; i <
				 * entry.getValue().size(); i++) { DataframeOperation leaf =
				 * operand.getLeaf(); operand = operand.prefix(); if (leaf
				 * instanceof AccessOperation<?>) leaf = new
				 * ProjectionOperation(leaf.getWhere(), ((AccessOperation<?>)
				 * leaf).getSelection()); ws.push(leaf); } DataframeOperation
				 * leaf = operand.getLeaf(); while (!ws.isEmpty()) {
				 * DataframeOperation popped = ws.pop();
				 * operand.addNode(popped); operand.addEdge(new SimpleEdge(leaf,
				 * popped)); leavesWs.push(popped); } leaves.put(entry.getKey(),
				 * leavesWs); for (int pos : entry.getValue()) operands[pos] =
				 * operand; } } for (int i = 0; i < operands.length; i++) {
				 * DataframeGraph graph = operands[i]; DataframeOperation exit =
				 * leaves.get(opIndexes[i]).pop(); concatGraph.mergeWith(graph);
				 * concatGraph.addEdge(new ConcatEdge(exit, concatNode, i)); }
				 * return new DFOrConstant(new
				 * DataframeGraphDomain(concatGraph)); } else
				 */
			return arg.top();
		}

		@Override
		public DataframeGraphDomain visit(BinaryExpression expression, DataframeGraphDomain left,
				DataframeGraphDomain right, Object... params) throws SemanticException {
			if (left.isBottom())
				return left;
			if (right.isBottom())
				return right;

			/*
			 * if (expression.getOperator() == TypeCast.INSTANCE) return
			 * evalTypeCast(expression, left, right, (ProgramPoint) params[1]);
			 * if (expression.getOperator() == TypeConv.INSTANCE) return
			 * evalTypeConv(expression, left, right, (ProgramPoint) params[1]);
			 * return evalBinaryExpression(expression.getOperator(), left,
			 * right, (ProgramPoint) params[1]);
			 */
			return top();
		}

		@Override
		public DataframeGraphDomain visit(TernaryExpression expression, DataframeGraphDomain left,
				DataframeGraphDomain middle, DataframeGraphDomain right, Object... params)
				throws SemanticException {
			if (left.isBottom())
				return left;
			if (middle.isBottom())
				return middle;
			if (right.isBottom())
				return right;

			/*
			 * return evalTernaryExpression(expression.getOperator(), left,
			 * middle, right, (ProgramPoint) params[1]);
			 */
			return top();
		}

		@Override
		public DataframeGraphDomain visit(Skip expression, Object... params) throws SemanticException {
			return DataframeGraphDomain.this;
		}

		@Override
		public DataframeGraphDomain visit(PushAny expression, Object... params) throws SemanticException {
			ProgramPoint pp = (ProgramPoint) params[1];
			if (isDataframeRelated(expression))
				return new DataframeGraphDomain(
						constants.smallStepSemantics(expression, pp),
						graph,
						pointers.setStack(NO_IDS),
						operations.setStack(NO_NODES.top()));
			else
				return new DataframeGraphDomain(
						constants.smallStepSemantics(expression, pp),
						graph,
						pointers.setStack(NO_IDS),
						operations.setStack(NO_NODES));
		}

		@Override
		public DataframeGraphDomain visit(Constant expression, Object... params) throws SemanticException {
			return new DataframeGraphDomain(
					constants.smallStepSemantics(expression, (ProgramPoint) params[1]),
					graph,
					pointers.setStack(NO_IDS),
					operations.setStack(NO_NODES));
		}

		@Override
		public DataframeGraphDomain visit(Identifier expression, Object... params) throws SemanticException {
			ProgramPoint pp = (ProgramPoint) params[1];
			if (constants.getKeys().contains(expression))
				return new DataframeGraphDomain(
						constants.smallStepSemantics(expression, pp),
						graph,
						pointers.setStack(NO_IDS),
						operations.setStack(NO_NODES));
			else if (pointers.getKeys().contains(expression))
				return new DataframeGraphDomain(
						constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						graph,
						pointers.setStack(pointers.getState(expression)),
						operations.setStack(NO_NODES));
			else
				return new DataframeGraphDomain(
						constants.smallStepSemantics(new Skip(pp.getLocation()), pp),
						graph,
						pointers.setStack(NO_IDS),
						operations.setStack(NO_NODES));
		}
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
