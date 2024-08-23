package it.unive.pylisa.analysis.dataframes;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.ScopeToken;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.heap.pointbased.HeapAllocationSite;
import it.unive.lisa.analysis.heap.pointbased.StackAllocationSite;
import it.unive.lisa.analysis.lattices.Satisfiability;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.value.ValueDomain;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.PushInv;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.lisa.util.representation.ObjectRepresentation;
import it.unive.lisa.util.representation.SetRepresentation;
import it.unive.lisa.util.representation.StringRepresentation;
import it.unive.lisa.util.representation.StructuredRepresentation;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.edge.AssignEdge;
import it.unive.pylisa.analysis.dataframes.edge.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.edge.ConsumeEdge;
import it.unive.pylisa.analysis.dataframes.edge.DataframeEdge;
import it.unive.pylisa.analysis.dataframes.edge.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.operations.Assign;
import it.unive.pylisa.analysis.dataframes.operations.Concat;
import it.unive.pylisa.analysis.dataframes.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.operations.GetAxis;
import it.unive.pylisa.analysis.dataframes.operations.Init;
import it.unive.pylisa.analysis.dataframes.operations.Iteration;
import it.unive.pylisa.analysis.dataframes.operations.Project;
import it.unive.pylisa.analysis.dataframes.operations.Read;
import it.unive.pylisa.analysis.dataframes.operations.Reshape;
import it.unive.pylisa.analysis.dataframes.operations.Transform;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.AllColumns;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnIteration;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnListSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnRangeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.AllRows;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.ConditionalSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowRangeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;
import it.unive.pylisa.analysis.dataframes.symbolic.AccessKeys;
import it.unive.pylisa.analysis.dataframes.symbolic.AssignToConstant;
import it.unive.pylisa.analysis.dataframes.symbolic.AssignToSelection;
import it.unive.pylisa.analysis.dataframes.symbolic.AxisConcatenation;
import it.unive.pylisa.analysis.dataframes.symbolic.BinaryTransform;
import it.unive.pylisa.analysis.dataframes.symbolic.ColumnProjection;
import it.unive.pylisa.analysis.dataframes.symbolic.CopyDataframe;
import it.unive.pylisa.analysis.dataframes.symbolic.CreateDataframe;
import it.unive.pylisa.analysis.dataframes.symbolic.DataframeOperator;
import it.unive.pylisa.analysis.dataframes.symbolic.DataframeProjection;
import it.unive.pylisa.analysis.dataframes.symbolic.DropCols;
import it.unive.pylisa.analysis.dataframes.symbolic.Iterate;
import it.unive.pylisa.analysis.dataframes.symbolic.JoinCols;
import it.unive.pylisa.analysis.dataframes.symbolic.ReadDataframe;
import it.unive.pylisa.analysis.dataframes.symbolic.RowProjection;
import it.unive.pylisa.analysis.dataframes.symbolic.SeriesComparison;
import it.unive.pylisa.analysis.dataframes.symbolic.UnaryReshape;
import it.unive.pylisa.analysis.dataframes.symbolic.UnaryTransform;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.Axis;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.BinaryTransformKind;
import it.unive.pylisa.analysis.dataframes.symbolic.Enumerations.UnaryTransformKind;
import it.unive.pylisa.analysis.dataframes.symbolic.aux.DataframeColumnSlice;
import it.unive.pylisa.analysis.dataframes.symbolic.aux.DataframeColumnSlice.ColumnSlice;
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

public class DataframeGraphDomain implements ValueDomain<DataframeGraphDomain> {

	private static final Logger LOG = LogManager.getLogger(DataframeGraphDomain.class);

	public static final SetLattice<NodeId> NO_IDS = new SetLattice<NodeId>().bottom();
	private static final SetLattice<DataframeOperation> NO_NODES = new SetLattice<DataframeOperation>().bottom();

	public final ValueEnvironment<ConstantPropagation> constants;

	public final ConstantPropagation constStack;

	public final DataframeForest graph;

	public final CollectingMapLattice<Identifier, NodeId> v;

	public final SetLattice<NodeId> vStack;

	public final CollectingMapLattice<NodeId, DataframeOperation> l;

	public DataframeGraphDomain() {
		this.constants = new ValueEnvironment<>(new ConstantPropagation()).top();
		this.graph = new DataframeForest(true).top();
		this.v = new CollectingMapLattice<Identifier, NodeId>(new SetLattice<>()).top();
		this.l = new CollectingMapLattice<NodeId, DataframeOperation>(new SetLattice<>()).top();
		this.constStack = this.constants.lattice.top();
		this.vStack = NO_IDS.top();
	}

	private DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> v,
			CollectingMapLattice<NodeId, DataframeOperation> l) {
		this(constants, constants.lattice.bottom(), graph, v, NO_IDS, l);
	}

	private DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> v,
			SetLattice<NodeId> vStack,
			CollectingMapLattice<NodeId, DataframeOperation> l) {
		this(constants, constants.lattice.bottom(), graph, v, vStack, l);
	}

	private DataframeGraphDomain(
			ValueEnvironment<ConstantPropagation> constants,
			ConstantPropagation constStack,
			DataframeForest graph,
			CollectingMapLattice<Identifier, NodeId> v,
			SetLattice<NodeId> vStack,
			CollectingMapLattice<NodeId, DataframeOperation> l) {
		super();
		this.constants = constants;
		this.constStack = constStack;
		this.graph = graph;
		this.v = v;
		this.vStack = vStack.isEmpty() ? NO_IDS : vStack;

		// cleanup unreachable nodes
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(l.getMap());
		if (map != null && !map.isEmpty()) {
			// get all the node ids used in the operations map
			Set<NodeId> nodes = new HashSet<>(l.getKeys());
			// remove all the ones used in the pointers map
			for (SetLattice<NodeId> used : this.v.getValues())
				used.forEach(nodes::remove);
			vStack.forEach(nodes::remove);
			// the remaining ones are unused: forget about them
			nodes.forEach(map::remove);
			this.l = new CollectingMapLattice<>(l.lattice, map);
		} else
			this.l = l;

		// FIXME temporary sanity check
//		SetLattice<DataframeOperation> pointed = resolvePointers(this);
//		for (DataframeOperation op : pointed)
//			if (!graph.containsNode(op))
//				throw new IllegalStateException();
	}

	@Override
	public DataframeGraphDomain assign(
			Identifier id,
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		DataframeGraphDomain sss = smallStepSemantics(expression, pp, oracle);
		if (!sss.constStack.isBottom())
			return new DataframeGraphDomain(
					sss.constants.putState(id, sss.constStack),
					sss.graph,
					sss.v,
					sss.l);
		else if (!isDataframeRelated(expression, pp, oracle))
			return sss;
		else if (!sss.vStack.isBottom())
			return new DataframeGraphDomain(
					sss.constants,
					sss.graph,
					sss.v.putState(id, sss.vStack),
					sss.vStack,
					sss.l);
		else
			return sss;
	}

	@Override
	public DataframeGraphDomain smallStepSemantics(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (expression instanceof Identifier)
			return visit((Identifier) expression, pp);
		else if (expression instanceof Constant)
			return visit((Constant) expression, pp, oracle);
		else if (expression instanceof PushAny)
			return visit((PushAny) expression, pp, oracle);
		else if (expression instanceof UnaryExpression) {
			UnaryExpression unary = (UnaryExpression) expression;
			DataframeGraphDomain arg = smallStepSemantics((ValueExpression) unary.getExpression(), pp, oracle);
			return arg.visit(unary, arg, pp, oracle);
		} else if (expression instanceof BinaryExpression) {
			BinaryExpression binary = (BinaryExpression) expression;
			DataframeGraphDomain left = smallStepSemantics((ValueExpression) binary.getLeft(), pp, oracle);
			DataframeGraphDomain right = left.smallStepSemantics((ValueExpression) binary.getRight(), pp, oracle);
			return right.visit(binary, left, right, pp, oracle);
		} else if (expression instanceof TernaryExpression) {
			TernaryExpression ternary = (TernaryExpression) expression;
			DataframeGraphDomain left = smallStepSemantics((ValueExpression) ternary.getLeft(), pp, oracle);
			DataframeGraphDomain middle = left.smallStepSemantics((ValueExpression) ternary.getMiddle(), pp, oracle);
			DataframeGraphDomain right = middle.smallStepSemantics((ValueExpression) ternary.getRight(), pp, oracle);
			return right.visit(ternary, left, middle, right, pp, oracle);
		}
		return this;
	}

	@Override
	public DataframeGraphDomain assume(
			ValueExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants.assume(expression, src, dest, oracle),
				graph,
				v,
				l);
	}

	@Override
	public DataframeGraphDomain forgetIdentifier(
			Identifier id)
			throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> v2 = this.v.lift(i -> id.equals(i) ? null : i, e -> e);
		return new DataframeGraphDomain(
				constants.forgetIdentifier(id),
				graph,
				v2,
				l);
	}

	@Override
	public DataframeGraphDomain forgetIdentifiersIf(
			Predicate<Identifier> test)
			throws SemanticException {
		CollectingMapLattice<Identifier, NodeId> v2 = this.v.lift(id -> test.test(id) ? null : id, e -> e);
		return new DataframeGraphDomain(
				constants.forgetIdentifiersIf(test),
				graph,
				v2,
				l);
	}

	@Override
	public Satisfiability satisfies(
			ValueExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		Satisfiability c = constants.satisfies(expression, pp, oracle);
		if (c == Satisfiability.SATISFIED || c == Satisfiability.NOT_SATISFIED)
			return c;
		return Satisfiability.UNKNOWN;
	}

	@Override
	public DataframeGraphDomain pushScope(
			ScopeToken token)
			throws SemanticException {
		return this;/*
					 * new DataframeGraphDomain( constants.pushScope(token),
					 * graph, pointers.lift(id -> (Identifier)
					 * id.pushScope(token), e -> e), operations);
					 */
	}

	@Override
	public DataframeGraphDomain popScope(
			ScopeToken token)
			throws SemanticException {
		return this;/*
					 * new DataframeGraphDomain( constants.popScope(token),
					 * graph, pointers.lift(id -> (Identifier)
					 * id.popScope(token), e -> e), operations);
					 */
	}

	@Override
	public StructuredRepresentation representation() {
		return new ObjectRepresentation(Map.of(
				"constants", constants.representation(),
				"constants-stack", constStack.representation(),
				"v", v.representation(StringRepresentation::new),
				"v-stack", new SetRepresentation(vStack.elements(), StringRepresentation::new),
				"l", l.representation(StringRepresentation::new),
				"graph", graph.representation()));
	}

	@Override
	public DataframeGraphDomain lub(
			DataframeGraphDomain other)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants.lub(other.constants),
				constStack.lub(other.constStack),
				graph.lub(other.graph),
				v.lub(other.v),
				vStack.lub(other.vStack),
				l.lub(other.l));
	}

	@Override
	public DataframeGraphDomain widening(
			DataframeGraphDomain other)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants.widening(other.constants),
				constStack.widening(other.constStack),
				graph.widening(other.graph),
				v.widening(other.v),
				vStack.widening(other.vStack),
				l.widening(other.l));
	}

	@Override
	public boolean lessOrEqual(
			DataframeGraphDomain other)
			throws SemanticException {
		return constants.lessOrEqual(other.constants)
				&& constStack.lessOrEqual(other.constStack)
				&& graph.lessOrEqual(other.graph)
				&& v.lessOrEqual(other.v)
				&& vStack.lessOrEqual(other.vStack)
				&& l.lessOrEqual(other.l);
	}

	@Override
	public DataframeGraphDomain top() {
		return new DataframeGraphDomain(
				constants.top(),
				constStack.top(),
				graph.top(),
				v.top(),
				NO_IDS,
				l.top());
	}

	@Override
	public boolean isTop() {
		return constants.isTop()
				&& constStack.isTop()
				&& graph.isTop()
				&& v.isTop()
				&& vStack == NO_IDS
				&& l.isTop();
	}

	@Override
	public DataframeGraphDomain bottom() {
		return new DataframeGraphDomain(
				constants.bottom(),
				constStack.bottom(),
				graph.bottom(),
				v.bottom(),
				NO_IDS,
				l.bottom());
	}

	@Override
	public boolean isBottom() {
		return constants.isBottom()
				&& constStack.isBottom()
				&& graph.isBottom()
				&& v.isBottom()
				&& vStack == NO_IDS
				&& l.isBottom();
	}

	public static boolean isDataframeRelated(
			SymbolicExpression expression,
			ProgramPoint pp,
			SemanticOracle oracle) {
		if (expression instanceof PushInv)
			// the type approximation of a pushinv is bottom, so the below check
			// will always fail regardless of the kind of value we are tracking
			return PyLibraryUnitType.is(expression.getStaticType(), LibrarySpecificationProvider.PANDAS, false);

		Set<Type> rts = null;
		try {
			rts = oracle.getRuntimeTypesOf(expression, pp, oracle);
		} catch (SemanticException e) {
			return false;
		}

		if (rts == null || rts.isEmpty())
			// if we have no runtime types, either the type domain has no type
			// information for the given expression (thus it can be anything,
			// also something that we can track) or the computation returned
			// bottom (and the whole state is likely going to go to bottom
			// anyway).
			return PyLibraryUnitType.is(expression.getStaticType(), LibrarySpecificationProvider.PANDAS, false)
					|| expression.getStaticType().isUntyped();

		return rts.stream().anyMatch(t -> PyLibraryUnitType.is(t, LibrarySpecificationProvider.PANDAS, false)
				|| expression.getStaticType().isUntyped());
	}

	private static boolean topOrBottom(
			Lattice<?> l) {
		return l.isTop() || l.isBottom();
	}

	private static SetLattice<DataframeOperation> resolvePointers(
			DataframeGraphDomain domain) {
		return resolvePointers(domain.l, domain.vStack);
	}

	private static SetLattice<DataframeOperation> resolvePointers(
			CollectingMapLattice<NodeId, DataframeOperation> l,
			SetLattice<NodeId> vStack) {
		if (topOrBottom(l) || topOrBottom(vStack))
			return NO_NODES;

		Set<DataframeOperation> resolved = new HashSet<>();
		for (NodeId pointer : vStack)
			resolved.addAll(l.getState(pointer).elements());
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

	private static RangeBound getRangeBound(
			ConstantPropagation c) {
		RangeBound bound;
		if (c.isTop())
			bound = null;
		else if (c.is(Integer.class))
			bound = new RangeBound(c.as(Integer.class));
		else
			bound = c.as(RangeBound.class);
		return bound;
	}

	private static ColumnRangeSelection getRangeBound(
			SetLattice<DataframeOperation> ops)
			throws SemanticException {
		ColumnRangeSelection bound = ColumnRangeSelection.BOTTOM;

		for (DataframeOperation op : ops) {
			if (!(op instanceof Project<?, ?>))
				return ColumnRangeSelection.TOP;

			ColumnSelection<?> selection = ((Project<?, ?>) op).getSelection().getColumnSelection();
			if (!(selection instanceof ColumnRangeSelection))
				return ColumnRangeSelection.TOP;

			bound = bound.lub((ColumnRangeSelection) selection);
		}

		return bound;
	}

	public DataframeGraphDomain visit(
			UnaryExpression expression,
			DataframeGraphDomain arg,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (arg.isBottom())
			return arg;

		UnaryOperator operator = expression.getOperator();
		if (!(operator instanceof DataframeOperator))
			throw new SemanticException(operator.getClass() + " is not a dataframe operator");

		int index = ((DataframeOperator) operator).getIndex();
		if (operator instanceof ReadDataframe)
			return doReadDataframe(index, arg, pp);
		if (operator instanceof CreateDataframe)
			return doCreateDataframe(index, arg, pp);
		if (operator instanceof CopyDataframe)
			return doCopyDataframe(index, arg, pp);
		if (operator instanceof UnaryTransform)
			return doUnaryTransformation(index, arg, operator, pp);
		if (operator instanceof UnaryReshape)
			return doUnaryReshape(index, arg, operator, pp);
		if (operator instanceof AxisConcatenation)
			return doAxisConcatenation(index, arg, operator, pp);
		if (operator instanceof AccessKeys)
			return doAccessKeys(index, arg, pp);
		if (operator instanceof Iterate)
			return doIterate(index, arg, pp);
		if (!arg.constStack.isBottom()
				&& arg.constants.lattice.canProcess(expression, pp, oracle))
			return delegateToConstants(expression, arg, pp, oracle);

		return cleanStack(arg, pp);
	}

	private static DataframeGraphDomain delegateToConstants(
			ValueExpression expression,
			DataframeGraphDomain arg,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return new DataframeGraphDomain(
				arg.constants,
				arg.constStack.eval(expression, arg.constants, pp, oracle),
				arg.graph,
				arg.v,
				NO_IDS,
				arg.l);
	}

	private static DataframeGraphDomain doAccessKeys(
			int index,
			DataframeGraphDomain arg,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(arg);
		if (topOrBottom(ops))
			return cleanStack(arg, pp);

		GetAxis access = new GetAxis(pp.getLocation(), index, Axis.COLS);
		DataframeForest forest = new DataframeForest(arg.graph);
		forest.addNode(access);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, access));

		NodeId id = new NodeId(access);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.v, arg.vStack, ids);
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				pointers,
				ids,
				arg.l.putState(id, new SetLattice<>(access)));
	}

	private static DataframeGraphDomain doIterate(
			int index,
			DataframeGraphDomain arg,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(arg);
		if (topOrBottom(ops))
			return cleanStack(arg, pp);

		Iteration access = new Iteration(pp.getLocation(), index);
		DataframeForest forest = new DataframeForest(arg.graph);
		forest.addNode(access);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, access));

		NodeId id = new NodeId(access);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.v, arg.vStack, ids);
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				pointers,
				ids,
				arg.l.putState(id, new SetLattice<>(access)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doAxisConcatenation(
			int index,
			DataframeGraphDomain arg,
			UnaryOperator operator,
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
						arg.v,
						(SetLattice<NodeId>) firstWrapped,
						arg.l);
			else
				return cleanStack(arg, pp);

		DataframeForest forest = new DataframeForest(arg.graph);
		DataframeOperation concatNode = new Concat(pp.getLocation(), index,
				operator instanceof JoinCols ? Axis.COLS : Axis.ROWS);
		forest.addNode(concatNode);

		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.l.getMap());

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
			positions.computeIfAbsent(idx, ii -> new LinkedList<>()).add(i);
		}

		SetLattice<DataframeOperation>[] operands = new SetLattice[nelements];
		for (Entry<Integer, List<Integer>> entry : positions.entrySet()) {
			Lattice<?> element = distinctElements.get(entry.getKey());
			if (!(element instanceof SetLattice<?>))
				return cleanStack(arg, pp);

			SetLattice<DataframeOperation> ops = resolvePointers(arg.l, (SetLattice<NodeId>) element);
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
					for (DataframeOperation op : operand)
						fixed.add(op);

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
				DataframeOperation> ops = new CollectingMapLattice<>(arg.l.lattice, operations);
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				forest,
				arg.v,
				new SetLattice<>(id),
				ops.putState(id, new SetLattice<>(concatNode)));
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doUnaryTransformation(
			int index,
			DataframeGraphDomain arg,
			UnaryOperator operator,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> stack = resolvePointers(arg);
		if (topOrBottom(stack))
			return cleanStack(arg, pp);

		UnaryTransform op = (UnaryTransform) operator;

		DataframeForest df = new DataframeForest(arg.graph);
		Map<NodeId, DataframeOperation> ids = new HashMap<>();
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(arg.l.getMap());
		for (DataframeOperation leaf : stack) {
			DataframeSelection<?, ?> selection;
			if (leaf instanceof Project<?, ?>)
				selection = ((Project<?, ?>) leaf).getSelection();
			else
				selection = new DataframeSelection<>(true);

			Transform<?, ?> t = op.getArg().isPresent()
					? new Transform(pp.getLocation(), index, op.getKind(), op.getAxis(),
							selection, op.getArg().get())
					: new Transform(pp.getLocation(), index, op.getKind(), op.getAxis(),
							selection);

			df.addNode(t);
			if (leaf instanceof Project<?, ?>)
				df.addEdge(new ConsumeEdge(leaf, t));
			else
				df.addEdge(new SimpleEdge(leaf, t));
			NodeId id = new NodeId(t);
			ids.put(id, t);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(arg.l.lattice, map);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.v, arg.vStack, idsLattice);
		return new DataframeGraphDomain(
				arg.constants,
				df,
				pointers,
				idsLattice,
				ops);
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doUnaryReshape(
			int index,
			DataframeGraphDomain arg,
			UnaryOperator operator,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> stack = resolvePointers(arg);
		if (topOrBottom(stack))
			return cleanStack(arg, pp);

		UnaryReshape op = (UnaryReshape) operator;

		DataframeForest df = new DataframeForest(arg.graph);
		Map<NodeId, DataframeOperation> ids = new HashMap<>();
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(arg.l.getMap());
		for (DataframeOperation leaf : stack) {
			DataframeSelection<?, ?> selection;
			if (leaf instanceof Project<?, ?>)
				selection = ((Project<?, ?>) leaf).getSelection();
			else
				selection = new DataframeSelection<>(true);

			Reshape<?, ?> r = new Reshape(pp.getLocation(), index, op.getKind(), selection);

			df.addNode(r);
			if (leaf instanceof Project<?, ?>)
				df.addEdge(new ConsumeEdge(leaf, r));
			else
				df.addEdge(new SimpleEdge(leaf, r));
			NodeId id = new NodeId(r);
			ids.put(id, r);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(arg.l.lattice, map);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(arg.v, arg.vStack, idsLattice);
		return new DataframeGraphDomain(
				arg.constants,
				df,
				pointers,
				idsLattice,
				ops);
	}

	private static DataframeGraphDomain doCopyDataframe(
			int index,
			DataframeGraphDomain arg,
			ProgramPoint pp)
			throws SemanticException {
		if (topOrBottom(arg.vStack))
			return cleanStack(arg, pp);

		Set<NodeId> ids = new HashSet<>();
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(arg.l.getMap());
		for (NodeId id : arg.vStack) {
			NodeId copy = new NodeId(id, index);
			ids.add(copy);
			operations.put(copy, arg.l.getState(id));
		}

		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				arg.graph,
				arg.v,
				new SetLattice<>(ids, false),
				new CollectingMapLattice<>(arg.l.lattice, operations));
	}

	private static DataframeGraphDomain doReadDataframe(
			int index,
			DataframeGraphDomain arg,
			ProgramPoint pp)
			throws SemanticException {
		ConstantPropagation filename = arg.constStack;
		DataframeForest df = new DataframeForest(arg.graph);
		Read op = new Read(pp.getLocation(), index, filename);
		df.addNode(op);

		NodeId id = new NodeId(op);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		SetLattice<DataframeOperation> operations = new SetLattice<>(op);
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				df,
				arg.v,
				ids,
				arg.l.putState(id, operations));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doCreateDataframe(
			int index,
			DataframeGraphDomain arg,
			ProgramPoint pp)
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
		// TODO rows
		Init op = new Init(pp.getLocation(), index, new Names(cols),
				new NumberSlice(new ConstantPropagation(0), new ConstantPropagation().top()));
		df.addNode(op);

		NodeId id = new NodeId(op);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		SetLattice<DataframeOperation> operations = new SetLattice<>(op);
		// no shift necessary: this is a new dataframe creation
		return new DataframeGraphDomain(
				arg.constants,
				df,
				arg.v,
				ids,
				arg.l.putState(id, operations));
	}

	public DataframeGraphDomain visit(
			BinaryExpression expression,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (left.isBottom())
			return left;
		if (right.isBottom())
			return right;

		BinaryOperator operator = expression.getOperator();

		if (operator == ListAppend.INSTANCE)
			return doListAppend(left, right, pp, oracle);

		if (!(operator instanceof DataframeOperator))
			throw new SemanticException(operator.getClass() + " is not a dataframe operator");

		int index = ((DataframeOperator) operator).getIndex();
		if (operator instanceof ColumnProjection)
			return doColumnAccess(index, left, right, pp);
		if (operator instanceof BinaryTransform)
			return doBinaryTransformation(index, left, right, operator, pp);
		if (operator instanceof DropCols)
			return doDropColumns(index, left, right, pp);
		if (operator instanceof JoinCols)
			return doJoinColumns(index, left, right, pp);
		if (operator instanceof AssignToSelection)
			return doWriteSelectionDataframe(index, left, right, pp);
		if (operator instanceof AssignToConstant)
			return doWriteSelectionConstant(index, left, right, pp);
		if (operator instanceof SeriesComparison)
			return doPandasSeriesComparison(index, left, right, pp, operator);
		if (!left.constStack.isBottom()
				&& !right.constStack.isBottom()
				&& right.constants.lattice.canProcess(expression, pp, oracle))
			return delegateToConstants(expression, right, pp, oracle);

		return cleanStack(right, pp);
	}

	private static DataframeGraphDomain doPandasSeriesComparison(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp,
			BinaryOperator operator)
			throws SemanticException {
		SetLattice<DataframeOperation> df1 = resolvePointers(left);
		ConstantPropagation value = right.constStack;
		if (topOrBottom(df1))
			return cleanStack(right, pp);
		if (value.isBottom()) // unknown variables can lead to top
			value = value.top();

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.l.getMap());
		SeriesComparison seriesCompOp = (SeriesComparison) operator;
		Map<NodeId, DataframeOperation> ids = new HashMap<>();

		for (DataframeOperation op : df1) {
			if (!(op instanceof Project<?, ?>))
				return cleanStack(right, pp);

			Project<?, ?> projection = (Project<?, ?>) op;
			if (!(projection.getSelection().getColumnSelection() instanceof ColumnListSelection))
				return cleanStack(right, pp);

			ConditionalSelection booleanSelection = new ConditionalSelection(
					(ColumnListSelection) projection.getSelection().getColumnSelection(), seriesCompOp.getOp(), value);
			Project<?, ?> boolComp = new Project<>(pp.getLocation(), index,
					booleanSelection);

			forest.addNode(boolComp);
			forest.addEdge(new ConsumeEdge(op, boolComp));
			NodeId id = new NodeId(boolComp);
			ids.put(id, boolComp);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.l.lattice, operations);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, idsLattice);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				idsLattice,
				ops);
	}

	private static DataframeGraphDomain doWriteSelectionConstant(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> df = resolvePointers(left);
		ConstantPropagation c = right.constStack;
		if (topOrBottom(df) || topOrBottom(c))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.l.getMap());
		Map<NodeId, DataframeOperation> ids = new HashMap<>();

		for (DataframeOperation op : df) {
			if (!(op instanceof Project<?, ?>))
				return cleanStack(right, pp);

			Project<?, ?> access = (Project<?, ?>) op;
			DataframeSelection<?, ?> selection = access.getSelection();
			DataframeOperation nodeToAdd = new Transform<>(pp.getLocation(), index, BinaryTransformKind.ASSIGN,
					Axis.ROWS, selection, c);
			forest.addNode(nodeToAdd);
			forest.addEdge(new ConsumeEdge(op, nodeToAdd));
			NodeId id = new NodeId(nodeToAdd);
			ids.put(id, nodeToAdd);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.l.lattice, operations);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, idsLattice);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				idsLattice,
				ops);
	}

	@SuppressWarnings({ "rawtypes", "unchecked" })
	private static DataframeGraphDomain doWriteSelectionDataframe(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> df1 = resolvePointers(left);
		SetLattice<DataframeOperation> df2 = resolvePointers(right);
		if (topOrBottom(df1) || topOrBottom(df2))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.l.getMap());
		Map<NodeId, DataframeOperation> ids = new HashMap<>();

		for (DataframeOperation op : df1) {
			if (!(op instanceof Project<?, ?>))
				return cleanStack(right, pp);

			Assign assign = new Assign(pp.getLocation(), index,
					((Project<?, ?>) op).getSelection());
			forest.addNode(assign);
			forest.addEdge(new ConsumeEdge(op, assign));
			NodeId id = new NodeId(assign);
			ids.put(id, assign);
		}

		for (DataframeOperation op : df2)
			for (DataframeOperation assign : ids.values())
				forest.addEdge(new AssignEdge(op, assign));

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.l.lattice, operations);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, idsLattice);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				idsLattice,
				ops);
	}

	private static DataframeGraphDomain doJoinColumns(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> df1 = resolvePointers(left);
		SetLattice<DataframeOperation> df2 = resolvePointers(right);
		if (topOrBottom(df1) || topOrBottom(df2))
			return cleanStack(right, pp);

		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.l.getMap());

		DataframeOperation concatNode = new Concat(pp.getLocation(), index, Axis.COLS);
		forest.addNode(concatNode);

		for (DataframeOperation op : df1)
			forest.addEdge(new ConcatEdge(op, concatNode, 0));

		for (DataframeOperation op : df2)
			forest.addEdge(new ConcatEdge(op, concatNode, 1));

		// safety measure: remove edges if they exist
		DataframeEdge edge;
		for (DataframeOperation op1 : df1)
			for (DataframeOperation op2 : df2)
				if ((edge = forest.getEdgeConnecting(op1, op2)) != null)
					forest.getNodeList().removeEdge(edge);

		NodeId id = new NodeId(concatNode);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<NodeId,
				DataframeOperation> ops = new CollectingMapLattice<>(right.l.lattice, operations);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				ids,
				ops.putState(id, new SetLattice<>(concatNode)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doDropColumns(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(left);
		ConstantPropagation cols = right.constStack;
		if (topOrBottom(ops) || topOrBottom(cols) || !cols.is(List.class))
			return cleanStack(right, pp);

		List<ConstantPropagation> cs = cols.as(List.class);
		Set<String> accessedCols = new HashSet<>();
		ColumnSelection<?> colsSelection = null;
		for (ConstantPropagation c : cs) {
			if (topOrBottom(c) || !c.is(String.class)) {
				colsSelection = AllColumns.INSTANCE;
				break;
			}
			accessedCols.add(c.as(String.class));
		}
		if (colsSelection == null)
			colsSelection = new ColumnListSelection(accessedCols);

		@SuppressWarnings("rawtypes")
		Transform<?, ?> drop = new Transform(pp.getLocation(), index, UnaryTransformKind.DROP_COLS, Axis.COLS,
				new DataframeSelection(colsSelection));
		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(drop);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, drop));

		NodeId id = new NodeId(drop);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				ids,
				right.l.putState(id, new SetLattice<>(drop)));
	}

	private static DataframeGraphDomain doBinaryTransformation(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			BinaryOperator operator,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> stack = resolvePointers(left);
		ConstantPropagation value = right.constStack;
		if (topOrBottom(stack))
			return cleanStack(right, pp);

		BinaryTransform op = (BinaryTransform) operator;
		DataframeForest forest = new DataframeForest(right.graph);
		Map<NodeId, DataframeOperation> ids = new HashMap<>();
		Map<NodeId, SetLattice<DataframeOperation>> map = new HashMap<>(left.l.getMap());
		for (DataframeOperation leaf : stack) {
			DataframeSelection<?, ?> selection;
			if (leaf instanceof Project<?, ?>)
				selection = ((Project<?, ?>) leaf).getSelection();
			else
				selection = new DataframeSelection<>(true);

			if (op.getArg().isPresent())
				throw new SemanticException("duplicate argument not supported");
			Transform<?, ?> t = new Transform<>(pp.getLocation(), index, op.getKind(), op.getAxis(), selection, value);

			forest.addNode(t);
			if (leaf instanceof Project<?, ?>)
				forest.addEdge(new ConsumeEdge(leaf, t));
			else
				forest.addEdge(new SimpleEdge(leaf, t));
			NodeId id = new NodeId(t);
			ids.put(id, t);
		}

		SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, idsLattice);
		CollectingMapLattice<NodeId, DataframeOperation> ops = new CollectingMapLattice<>(left.l.lattice, map);
		for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
			ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				idsLattice,
				ops);
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doColumnAccess(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
		SetLattice<DataframeOperation> ops = resolvePointers(left);
		SetLattice<DataframeOperation> args = resolvePointers(right);
		ConstantPropagation col = right.constStack;
		if (topOrBottom(ops))
			return cleanStack(right, pp);

		ColumnSelection<?> columns = null;
		RowSelection<?> rows = null;
		if (!topOrBottom(args)) {
			for (DataframeOperation arg : args)
				if (arg instanceof Iteration)
					columns = columns == null
							? new ColumnIteration()
							: columns.lub(new ColumnIteration());
				else if (arg instanceof Project<?, ?>) {
					columns = columns == null
							? ((Project<?, ?>) arg).getSelection().getColumnSelection()
							: columns.lub(((Project<?, ?>) arg).getSelection().getColumnSelection());
					rows = rows == null
							? ((Project<?, ?>) arg).getSelection().getRowSelection()
							: rows.lub(((Project<?, ?>) arg).getSelection().getRowSelection());
				} else
					return cleanStack(right, pp);
		} else if (topOrBottom(col))
			columns = AllColumns.INSTANCE;
		else if (col.is(String.class))
			columns = new ColumnListSelection(new Names(col.as(String.class)));
		else if (col.is(Integer.class))
			columns = new ColumnRangeSelection(col.as(Integer.class));
		else if (col.is(NumberSlice.class))
			columns = new ColumnRangeSelection(col.as(NumberSlice.class));
		else if (col.is(SliceConstant.class)) {
			SliceConstant c = col.as(SliceConstant.class);
			Slice slice = (Slice) c.getValue();
			columns = new ColumnRangeSelection(new NumberSlice(
					slice.getStart().toConstant(),
					slice.getEnd().toConstant(),
					slice.getSkip().toConstant()));
		} else if (col.is(List.class)) {
			List<ConstantPropagation> cs = col.as(List.class);
			Set<String> accessedCols = new HashSet<>();
			for (ConstantPropagation c : cs) {
				if (topOrBottom(c) || !c.is(String.class)) {
					columns = AllColumns.INSTANCE;
					break;
				}
				accessedCols.add(c.as(String.class));
			}
			columns = new ColumnListSelection(accessedCols);
		} else
			columns = AllColumns.INSTANCE;

		if (columns == null)
			columns = AllColumns.INSTANCE;
		if (rows == null)
			rows = AllRows.INSTANCE;
		Project<?, ?> access = new Project(pp.getLocation(), index, rows, columns);
		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(access);
		for (DataframeOperation op : ops)
			forest.addEdge(new SimpleEdge(op, access));
		if (!topOrBottom(args))
			for (DataframeOperation arg : args)
				forest.addEdge(new ConsumeEdge(arg, access));

		NodeId id = new NodeId(access);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				ids,
				right.l.putState(id, new SetLattice<>(access)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doListAppend(
			DataframeGraphDomain left,
			DataframeGraphDomain right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ConstantPropagation list = left.constStack;
		if (topOrBottom(list) || topOrBottom(right) || !list.is(List.class))
			return right;

		Lattice<?> tail;
		if (!right.constStack.isBottom())
			tail = right.constStack;
		else if (!right.vStack.isBottom())
			tail = right.vStack;
		else
			tail = right.constants.lattice.top();

		ListConstant listconst = new ListConstant(pp.getLocation(), list.as(List.class), tail);
		return new DataframeGraphDomain(
				right.constants,
				right.constStack.eval(listconst, right.constants, pp, oracle),
				right.graph,
				right.v,
				NO_IDS,
				right.l);
	}

	public DataframeGraphDomain visit(
			TernaryExpression expression,
			DataframeGraphDomain left,
			DataframeGraphDomain middle,
			DataframeGraphDomain right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (left.isBottom())
			return left;
		if (middle.isBottom())
			return middle;
		if (right.isBottom())
			return right;

		TernaryOperator operator = expression.getOperator();

		if (operator == DictPut.INSTANCE)
			return doDictPut(left, middle, right, pp, oracle);
		if (operator instanceof SliceCreation)
			return doSliceCreation(left, middle, right, pp, oracle);

		if (!(operator instanceof DataframeOperator))
			throw new SemanticException(operator.getClass() + " is not a dataframe operator");

		int index = ((DataframeOperator) operator).getIndex();
		if (operator instanceof RowProjection)
			return doProjectRows(index, left, middle, right, pp, operator);
		if (operator instanceof DataframeProjection)
			return doAccessRowsColumns(index, left, middle, right, pp);
		if (!left.constStack.isBottom()
				&& !middle.constStack.isBottom()
				&& !right.constStack.isBottom()
				&& right.constants.lattice.canProcess(expression, pp, oracle))
			return delegateToConstants(expression, right, pp, oracle);

		return cleanStack(right, pp);
	}

	private static DataframeGraphDomain doSliceCreation(
			DataframeGraphDomain left,
			DataframeGraphDomain middle,
			DataframeGraphDomain right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (right.constStack.isBottom())
			return cleanStack(right, pp);
		RangeBound skip = getRangeBound(right.constStack);
		RangeBound rstart = null, rend = null;
		ColumnRangeSelection cstart = null, cend = null;
		SetLattice<DataframeOperation> l = null, m = null;

		if (!left.constStack.isBottom())
			rstart = getRangeBound(left.constStack);
		else if (!topOrBottom(left.vStack)) {
			l = resolvePointers(left);
			cstart = getRangeBound(l);
		}

		if (!middle.constStack.isBottom())
			rend = getRangeBound(middle.constStack);
		else if (!topOrBottom(middle.vStack)) {
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
				right.constStack.eval(slice, right.constants, pp, oracle),
				right.graph,
				right.v,
				NO_IDS,
				right.l);
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	private static DataframeGraphDomain doAccessRowsColumns(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain middle,
			DataframeGraphDomain right,
			ProgramPoint pp)
			throws SemanticException {
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
					colsSelection = AllColumns.INSTANCE;
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
			RowRangeSelection rowsSelection = new RowRangeSelection(new NumberSlice(
					rowSlice.getStart() == null ? new ConstantPropagation().bottom() : rowSlice.getStart().toConstant(),
					rowSlice.getEnd() == null ? new ConstantPropagation().bottom() : rowSlice.getEnd().toConstant(),
					rowSlice.getSkip() == null ? new ConstantPropagation().bottom() : rowSlice.getSkip().toConstant()));
			DataframeSelection<?, ?> selection = new DataframeSelection(rowsSelection, colsSelection);
			DataframeOperation access = new Project<>(pp.getLocation(), index, selection);
			forest.addNode(access);
			for (DataframeOperation op : df)
				forest.addEdge(new SimpleEdge(op, access));

			for (DataframeOperation op : toConsume)
				forest.addEdge(new ConsumeEdge(op, access));

			NodeId id = new NodeId(access);
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier,
					NodeId> pointers = shift(right.v, left.vStack, ids);
			return new DataframeGraphDomain(
					right.constants,
					forest,
					pointers,
					ids,
					right.l.putState(id, new SetLattice<>(access)));
		} else if (!topOrBottom(middle.vStack)) {
			SetLattice<DataframeOperation> middleGraph = resolvePointers(middle);
			Map<NodeId, SetLattice<DataframeOperation>> operations = new HashMap<>(right.l.getMap());
			Map<NodeId, DataframeOperation> ids = new HashMap<>();

			for (DataframeOperation op : middleGraph) {
				if (!(op instanceof Project<?, ?>))
					return cleanStack(right, pp);

				Project<?, ?> colCompare = (Project<?, ?>) op;
				DataframeSelection<?, ?> selection = new DataframeSelection(
						colCompare.getSelection().getRowSelection(),
						colsSelection);
				DataframeOperation proj = new Project<>(pp.getLocation(), index, selection);

				forest.addNode(proj);
				forest.addEdge(new ConsumeEdge(op, proj));

				NodeId id = new NodeId(proj);
				ids.put(id, proj);

				for (DataframeOperation opc : toConsume)
					forest.addEdge(new ConsumeEdge(opc, proj));
			}

			SetLattice<NodeId> idsLattice = new SetLattice<>(ids.keySet(), false);
			CollectingMapLattice<NodeId,
					DataframeOperation> ops = new CollectingMapLattice<>(right.l.lattice, operations);
			for (Entry<NodeId, DataframeOperation> entry : ids.entrySet())
				ops = ops.putState(entry.getKey(), new SetLattice<>(entry.getValue()));
			CollectingMapLattice<Identifier,
					NodeId> pointers = shift(right.v, left.vStack, idsLattice);
			return new DataframeGraphDomain(
					right.constants,
					forest,
					pointers,
					idsLattice,
					ops);
		} else {
			DataframeSelection<?, ?> selection = new DataframeSelection<>(true);
			DataframeOperation access = new Project<>(pp.getLocation(), index, selection);
			forest.addNode(access);
			for (DataframeOperation op : df)
				forest.addEdge(new SimpleEdge(op, access));
			for (DataframeOperation op : toConsume)
				forest.addEdge(new ConsumeEdge(op, access));

			NodeId id = new NodeId(access);
			SetLattice<NodeId> ids = new SetLattice<>(id);
			CollectingMapLattice<Identifier,
					NodeId> pointers = shift(right.v, left.vStack, ids);
			return new DataframeGraphDomain(
					right.constants,
					forest,
					pointers,
					ids,
					right.l.putState(id, new SetLattice<>(access)));
		}
	}

	private static DataframeGraphDomain doProjectRows(
			int index,
			DataframeGraphDomain left,
			DataframeGraphDomain middle,
			DataframeGraphDomain right,
			ProgramPoint pp,
			TernaryOperator operator)
			throws SemanticException {
		SetLattice<DataframeOperation> df = resolvePointers(left);
		ConstantPropagation start = middle.constStack;
		ConstantPropagation end = right.constStack;
		if (topOrBottom(start) || topOrBottom(end) || topOrBottom(df))
			return cleanStack(right, pp);

		RowRangeSelection slice = new RowRangeSelection(
				new NumberSlice(start.as(Integer.class), end.as(Integer.class)));
		DataframeOperation node = new Project<>(pp.getLocation(), index, slice);
		DataframeForest forest = new DataframeForest(right.graph);
		forest.addNode(node);
		for (DataframeOperation op : df)
			forest.addEdge(new SimpleEdge(op, node));
		NodeId id = new NodeId(node);
		SetLattice<NodeId> ids = new SetLattice<>(id);
		CollectingMapLattice<Identifier, NodeId> pointers = shift(right.v, left.vStack, ids);
		return new DataframeGraphDomain(
				right.constants,
				forest,
				pointers,
				ids,
				right.l.putState(id, new SetLattice<>(node)));
	}

	@SuppressWarnings("unchecked")
	private static DataframeGraphDomain doDictPut(
			DataframeGraphDomain left,
			DataframeGraphDomain middle,
			DataframeGraphDomain right,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		ConstantPropagation dict = left.constStack;
		if (topOrBottom(dict) || topOrBottom(middle) || topOrBottom(right) || !dict.is(Map.class))
			return cleanStack(right, pp);

		Lattice<?> key, value;
		if (!middle.constStack.isBottom())
			key = middle.constStack;
		else if (!middle.vStack.isBottom())
			key = middle.vStack;
		else
			key = middle.constants.lattice.top();
		if (!right.constStack.isBottom())
			value = right.constStack;
		else if (!right.vStack.isBottom())
			value = right.vStack;
		else
			value = right.constants.lattice.top();

		DictConstant newdict = new DictConstant(pp.getLocation(), dict.as(Map.class), Pair.of(key, value));
		return new DataframeGraphDomain(
				right.constants,
				right.constStack.eval(newdict, right.constants, pp, oracle),
				right.graph,
				right.v,
				NO_IDS,
				right.l);
	}

	private static DataframeGraphDomain cleanStack(
			DataframeGraphDomain dom,
			ProgramPoint pp)
			throws SemanticException {
		LOG.debug("Evaluation of " + pp + " in " + getCaller() + " caused the stack to be cleaned");
		return new DataframeGraphDomain(
				dom.constants,
				dom.graph,
				dom.v,
				NO_IDS,
				dom.l);
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

	public DataframeGraphDomain visit(
			PushAny expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		if (isDataframeRelated(expression, pp, oracle))
			return new DataframeGraphDomain(
					constants,
					graph,
					v,
					NO_IDS.top(),
					l.top());
		else
			return new DataframeGraphDomain(
					constants,
					constStack.top(),
					graph,
					v,
					NO_IDS,
					l);
	}

	public DataframeGraphDomain visit(
			Constant expression,
			ProgramPoint pp,
			SemanticOracle oracle)
			throws SemanticException {
		return new DataframeGraphDomain(
				constants,
				constStack.eval(expression, constants, pp, oracle),
				graph,
				v,
				NO_IDS,
				l);
	}

	public DataframeGraphDomain visit(
			Identifier expression,
			ProgramPoint pp)
			throws SemanticException {
		if (expression instanceof MemoryPointer)
			expression = ((MemoryPointer) expression).getReferencedLocation();

		if (expression instanceof AllocationSite)
			expression = stripFields((AllocationSite) expression);

		if (constants.getKeys().contains(expression))
			return new DataframeGraphDomain(
					constants,
					constants.getState(expression),
					graph,
					v,
					NO_IDS,
					l);
		else if (v.getKeys().contains(expression))
			return new DataframeGraphDomain(
					constants,
					graph,
					v,
					v.getState(expression),
					l);
		else
			return new DataframeGraphDomain(
					constants,
					graph,
					v,
					NO_IDS,
					l);
	}

	public static AllocationSite stripFields(
			AllocationSite as) {
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
		return Objects.hash(constStack, constants, graph, l, v, vStack);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		DataframeGraphDomain other = (DataframeGraphDomain) obj;
		return Objects.equals(constStack, other.constStack)
				&& Objects.equals(constants, other.constants)
				&& Objects.equals(graph, other.graph)
				&& Objects.equals(l, other.l)
				&& Objects.equals(v, other.v)
				&& Objects.equals(vStack, other.vStack);
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
		for (SetLattice<NodeId> variable : v.getValues())
			for (NodeId id : variable)
				exits.addAll(l.getState(id).elements());

		for (DataframeOperation op : exits)
			result.addEdge(new SimpleEdge(op, close));

		return result;
	}

	public static class CloseOperation extends DataframeOperation {

		public CloseOperation() {
			super(SyntheticLocation.INSTANCE, -3);
		}

		@Override
		public String toString() {
			return "exit";
		}

		@Override
		protected DataframeOperation lubSameOperation(
				DataframeOperation other)
				throws SemanticException {
			return this;
		}

		@Override
		protected boolean lessOrEqualSameOperation(
				DataframeOperation other)
				throws SemanticException {
			return false;
		}

		@Override
		protected int compareToSameOperation(
				DataframeOperation o) {
			return 0;
		}
	}

	public Map<Identifier, DataframeForest> partitionByVarialbe() throws SemanticException {
		Map<Identifier, DataframeForest> result = new HashMap<>();

		for (Entry<Identifier, SetLattice<NodeId>> entry : v.getMap().entrySet())
			if (!entry.getValue().isTop() && !entry.getValue().isBottom()) {
				DataframeForest accumulator = null;
				for (NodeId id : entry.getValue())
					for (DataframeOperation op : l.getState(id)) {
						DataframeForest sub = graph.bDFS(op,
								o -> false,
								edge -> true);
						if (accumulator == null)
							accumulator = sub;
						else
							accumulator = accumulator.lub(sub);
					}
				result.put(entry.getKey(), accumulator);
			}

		return result;
	}

	@Override
	public boolean knowsIdentifier(
			Identifier id) {
		return constants.knowsIdentifier(id) || v.getKeys().contains(id);
	}
}
