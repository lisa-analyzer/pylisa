package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.heap.pointbased.AllocationSite;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.MemoryPointer;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;
import it.unive.lisa.util.collections.workset.FIFOWorkingSet;
import it.unive.lisa.util.collections.workset.LIFOWorkingSet;
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.Names;
import it.unive.pylisa.analysis.dataframes.transformation.graph.AssignEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.ConcatEdge;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeGraph;
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
import it.unive.pylisa.symbolic.operators.dataframes.DropCols;
import it.unive.pylisa.symbolic.operators.dataframes.FilterNull;
import it.unive.pylisa.symbolic.operators.dataframes.JoinCols;
import it.unive.pylisa.symbolic.operators.dataframes.PandasSeriesComparison;
import it.unive.pylisa.symbolic.operators.dataframes.PopSelection;
import it.unive.pylisa.symbolic.operators.dataframes.ProjectRows;
import it.unive.pylisa.symbolic.operators.dataframes.ReadDataframe;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionConstant;
import it.unive.pylisa.symbolic.operators.dataframes.WriteSelectionDataframe;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Collectors;
import org.apache.commons.lang3.tuple.Pair;

public class DFOrConstant extends BaseNonRelationalValueDomain<DFOrConstant> {

	private static final DFOrConstant TOP_GRAPH = new DFOrConstant(new DataframeGraphDomain().top());
	private static final DFOrConstant TOP_CONSTANT = new DFOrConstant(new ConstantPropagation().top());

	private static final DFOrConstant TOP = new DFOrConstant();

	private static final DFOrConstant BOTTOM = new DFOrConstant(new DataframeGraphDomain().bottom(),
			new ConstantPropagation().bottom());

	private final DataframeGraphDomain graph;

	private final ConstantPropagation constant;

	public DFOrConstant() {
		this(new DataframeGraphDomain().top(), new ConstantPropagation().top());
	}

	public DFOrConstant(DataframeGraphDomain graph) {
		this(graph, new ConstantPropagation().bottom());
	}

	public DFOrConstant(ConstantPropagation constant) {
		this(new DataframeGraphDomain().bottom(), constant);
	}

	private DFOrConstant(DataframeGraphDomain graph, ConstantPropagation constant) {
		this.graph = graph;
		this.constant = constant;
	}

	public ConstantPropagation constant() {
		return constant;
	}

	public DataframeGraphDomain df() {
		return graph;
	}

	@Override
	public DFOrConstant top() {
		return TOP;
	}

	@Override
	public boolean isTop() {
		return super.isTop() || (graph.isTop() && constant.isTop());
	}

	@Override
	public DFOrConstant bottom() {
		return BOTTOM;
	}

	@Override
	public boolean isBottom() {
		return super.isBottom() || (graph.isBottom() && constant.isBottom());
	}

	@Override
	protected DFOrConstant lubAux(DFOrConstant other) throws SemanticException {
		// bottom means it is not that kind of value: they need to be both
		// dataframes or constants
		if (graph.isBottom() != other.graph.isBottom())
			return TOP;
		if (constant.isBottom() != other.constant.isBottom())
			return TOP;

		// they cannot be both bottom, otherwise this would be bottom
		if (graph.isBottom())
			return new DFOrConstant(constant.lub(other.constant));
		else
			return new DFOrConstant(graph.lub(other.graph));
	}

	@Override
	protected DFOrConstant wideningAux(DFOrConstant other) throws SemanticException {
		// bottom means it is not that kind of value: they need to be both
		// dataframes or constants
		if (graph.isBottom() != other.graph.isBottom())
			return TOP;
		if (constant.isBottom() != !other.constant.isBottom())
			return TOP;

		// they cannot be both bottom, otherwise this would be bottom
		if (graph.isBottom())
			return new DFOrConstant(constant.widening(other.constant));
		else
			return new DFOrConstant(graph.widening(other.graph));
	}

	@Override
	protected boolean lessOrEqualAux(DFOrConstant other) throws SemanticException {
		// bottom means it is not that kind of value: they need to be both
		// dataframes or constants
		if (graph.isBottom() != other.graph.isBottom())
			return false;
		if (constant.isBottom() != !other.constant.isBottom())
			return false;

		// they cannot be both bottom, otherwise this would be bottom
		if (graph.isBottom())
			return constant.lessOrEqual(other.constant);
		else
			return graph.lessOrEqual(other.graph);
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + ((constant == null) ? 0 : constant.hashCode());
		result = prime * result + ((graph == null) ? 0 : graph.hashCode());
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
		DFOrConstant other = (DFOrConstant) obj;
		if (constant == null) {
			if (other.constant != null)
				return false;
		} else if (!constant.equals(other.constant))
			return false;
		if (graph == null) {
			if (other.graph != null)
				return false;
		} else if (!graph.equals(other.graph))
			return false;
		return true;
	}

	@Override
	public DomainRepresentation representation() {
		if (isTop())
			return Lattice.topRepresentation();
		if (isBottom())
			return Lattice.bottomRepresentation();
		if (graph.isBottom())
			return constant.representation();
		return graph.representation();
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return constant.tracksIdentifiers(id) || DataframeGraphDomain.tracks(id);
	}

	@Override
	public boolean canProcess(SymbolicExpression expression) {
		return constant.canProcess(expression) || DataframeGraphDomain.processes(expression);
	}

	private static boolean topOrBottom(Lattice<?> l) {
		return l.isTop() || l.isBottom();
	}

	@Override
	protected DFOrConstant evalIdentifier(Identifier id, ValueEnvironment<DFOrConstant> environment, ProgramPoint pp)
			throws SemanticException {
		if (id instanceof MemoryPointer)
			return environment.getState(((MemoryPointer) id).getReferencedLocation());

		if (id instanceof AllocationSite) {
			// TODO this is very fragile and only works with the current state
			// of the field sensitive program point based heap
			AllocationSite as = (AllocationSite) id;
			if (as.getName().endsWith("]"))
				id = new AllocationSite(as.getStaticType(), as.getLocationName(), as.isWeak(), as.getCodeLocation());
		}

		return super.evalIdentifier(id, environment, pp);
	}

	@Override
	protected DFOrConstant evalNonNullConstant(Constant constant, ProgramPoint pp) throws SemanticException {
		return new DFOrConstant(this.constant.evalNonNullConstant(constant, pp));
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	@Override
	protected DFOrConstant evalUnaryExpression(UnaryOperator operator, DFOrConstant arg, ProgramPoint pp)
			throws SemanticException {
		if (operator == ReadDataframe.INSTANCE) {
			ConstantPropagation filename = arg.constant;
			if (topOrBottom(filename))
				return TOP_GRAPH;

			DataframeGraphDomain df = new DataframeGraphDomain(
					new ReadFromFile(pp.getLocation(), filename.as(String.class)));
			return new DFOrConstant(df);
		} else if (operator instanceof ApplyTransformation) {
			DataframeGraphDomain df = arg.graph;
			if (topOrBottom(df))
				return TOP_GRAPH;

			ApplyTransformation op = (ApplyTransformation) operator;
			Kind kind = op.getKind();
			DataframeOperation leaf = df.getTransformations().getLeaf();
			if (!(leaf instanceof SelectionOperation<?>))
				return TOP_GRAPH;
			Transform t = op.getArg().isPresent()
					? new Transform(pp.getLocation(), kind, ((SelectionOperation<?>) leaf).getSelection(),
							op.getArg().get())
					: new Transform(pp.getLocation(), kind, ((SelectionOperation<?>) leaf).getSelection());

			DataframeGraphDomain conv = new DataframeGraphDomain(df.getTransformations().prefix(), t);
			return new DFOrConstant(conv);
		} else if (operator instanceof FilterNull) {
			DataframeGraphDomain df = arg.graph;
			if (topOrBottom(df))
				return TOP_GRAPH;

			DataframeGraphDomain dfNew = new DataframeGraphDomain(df,
					new FilterNullAxis(pp.getLocation(), ((FilterNull) operator).getAxis()));
			return new DFOrConstant(dfNew);
		} else if (operator instanceof PopSelection) {
			DataframeGraphDomain df = arg.graph;
			if (topOrBottom(df))
				return TOP_GRAPH;

			DataframeOperation leaf = df.getTransformations().getLeaf();
			if (!(leaf instanceof SelectionOperation<?>))
				return TOP_GRAPH;
			DataframeGraphDomain popped = new DataframeGraphDomain(df.getTransformations().prefix());
			return new DFOrConstant(popped);
		} else if (operator instanceof AxisConcatenation) {
			ConstantPropagation list = arg.constant;

			if (topOrBottom(list) || !list.is(List.class))
				return TOP_GRAPH;

			List<Lattice<?>> elements = list.as(List.class);

			if (elements.isEmpty())
				return BOTTOM;
			DFOrConstant firstWrapped = (DFOrConstant) elements.iterator().next();
			if (elements.size() == 1)
				return firstWrapped;

			DataframeGraph concatGraph = new DataframeGraph();
			DataframeOperation concatNode = new Concat(pp.getLocation(),
					operator == JoinCols.INSTANCE
							? Concat.Axis.CONCAT_COLS
							: Concat.Axis.CONCAT_ROWS);
			concatGraph.addNode(concatNode);

			// 1) if a leaf is an access it has to become a projection
			// 2) the same graph might appear multiple times as different nodes
			// of the concat

			int nelements = elements.size();
			List<Lattice<?>> distinctElements = elements.stream().distinct().collect(Collectors.toList());
			int[] opIndexes = new int[nelements];
			Map<Integer, List<Integer>> positions = new HashMap<>();
			for (int i = 0; i < nelements; i++) {
				int idx = distinctElements.indexOf(elements.get(i));
				opIndexes[i] = idx;
				positions.computeIfAbsent(idx, index -> new LinkedList<>()).add(i);
			}

			Map<Integer, LIFOWorkingSet<DataframeOperation>> leaves = new HashMap<>();
			DataframeGraph[] operands = new DataframeGraph[nelements];
			for (Entry<Integer, List<Integer>> entry : positions.entrySet()) {
				DataframeGraph original = ((DFOrConstant) distinctElements.get(entry.getKey())).graph
						.getTransformations();
				LIFOWorkingSet<DataframeOperation> leavesWs = LIFOWorkingSet.mk();
				if (entry.getValue().size() == 1) {
					// appears once
					operands[entry.getValue().iterator().next()] = original;
					leavesWs.push(original.getLeaf());
					leaves.put(entry.getKey(), leavesWs);
				} else {
					// appears more than once
					DataframeGraph operand = original;
					FIFOWorkingSet<DataframeOperation> ws = FIFOWorkingSet.mk();
					for (int i = 0; i < entry.getValue().size(); i++) {
						DataframeOperation leaf = operand.getLeaf();
						operand = operand.prefix();
						if (leaf instanceof AccessOperation<?>)
							leaf = new ProjectionOperation(leaf.getWhere(), ((AccessOperation<?>) leaf).getSelection());
						ws.push(leaf);
					}

					DataframeOperation leaf = operand.getLeaf();
					while (!ws.isEmpty()) {
						DataframeOperation popped = ws.pop();
						operand.addNode(popped);
						operand.addEdge(new SimpleEdge(leaf, popped));
						leavesWs.push(popped);
					}

					leaves.put(entry.getKey(), leavesWs);
					for (int pos : entry.getValue())
						operands[pos] = operand;
				}
			}

			for (int i = 0; i < operands.length; i++) {
				DataframeGraph graph = operands[i];
				DataframeOperation exit = leaves.get(opIndexes[i]).pop();
				concatGraph.mergeWith(graph);
				concatGraph.addEdge(new ConcatEdge(exit, concatNode, i));
			}

			return new DFOrConstant(new DataframeGraphDomain(concatGraph));
		} else
			return TOP;
	}

	@SuppressWarnings({ "rawtypes", "unchecked" })
	@Override
	protected DFOrConstant evalBinaryExpression(BinaryOperator operator, DFOrConstant left, DFOrConstant right,
			ProgramPoint pp) throws SemanticException {
		if (operator == ListAppend.INSTANCE) {
			ConstantPropagation list = left.constant;
			if (topOrBottom(list) || topOrBottom(right) || !list.is(List.class))
				return TOP_CONSTANT;

			ConstantPropagation newlist = new ConstantPropagation(
					new ListConstant(pp.getLocation(), list.as(List.class), right));

			return new DFOrConstant(newlist);
		} else if (operator == ColumnAccess.INSTANCE) {
			DataframeGraphDomain df = left.graph;
			ConstantPropagation col = right.constant;
			if (topOrBottom(df) || topOrBottom(col))
				return TOP_GRAPH;

			ColumnListSelection columns;
			if (col.is(String.class))
				columns = new ColumnListSelection(new Names(col.as(String.class)));
			else {
				List<DFOrConstant> cs = col.as(List.class);
				Set<String> accessedCols = new HashSet<>();

				for (DFOrConstant c : cs) {
					if (topOrBottom(c.constant) || !c.constant.is(String.class)) {
						columns = new ColumnListSelection(true);
						break;
					}
					accessedCols.add(c.constant.as(String.class));
				}
				columns = new ColumnListSelection(accessedCols);
			}

			DataframeGraphDomain ca = new DataframeGraphDomain(df,
					new AccessOperation<>(pp.getLocation(), columns));

			return new DFOrConstant(ca);
		} else if (operator == DropCols.INSTANCE) {
			DataframeGraphDomain df = left.graph;
			ConstantPropagation cols = right.constant;
			if (topOrBottom(df) || topOrBottom(cols) || !cols.is(List.class))
				return TOP_GRAPH;

			List<DFOrConstant> cs = cols.as(List.class);
			Set<String> accessedCols = new HashSet<>();
			ColumnListSelection colsSelection = null;

			for (DFOrConstant c : cs) {
				if (topOrBottom(c.constant) || !c.constant.is(String.class)) {
					colsSelection = new ColumnListSelection(true);
					break;
				}
				accessedCols.add(c.constant.as(String.class));
			}

			if (colsSelection == null)
				colsSelection = new ColumnListSelection(accessedCols);

			DataframeGraphDomain ca = new DataframeGraphDomain(df, new DropColumns(pp.getLocation(), colsSelection));
			return new DFOrConstant(ca);
		} else if (operator == JoinCols.INSTANCE) {
			DataframeGraphDomain df1 = left.graph;
			DataframeGraphDomain df2 = right.graph;

			if (topOrBottom(df1) || topOrBottom(df2))
				return TOP_GRAPH;

			if (df1.equals(df2)) {
				// the last two nodes are the leaves to use for writing
				DataframeOperation exit1 = null;
				DataframeOperation exit2 = null;
				DataframeGraph original = df1.getTransformations();
				DataframeGraph root = original.prefix();
				DataframeGraph preroot = root.prefix();

				try {
					exit1 = original.getLeaf();
					if (exit1 instanceof AccessOperation<?>)
						exit1 = new ProjectionOperation(exit1.getWhere(), ((AccessOperation<?>) exit1).getSelection());
					exit2 = root.getLeaf();
					if (exit2 instanceof AccessOperation<?>)
						exit2 = new ProjectionOperation(exit2.getWhere(), ((AccessOperation<?>) exit2).getSelection());
				} catch (IllegalStateException e) {
					// at least one has more than one leaf
					return TOP_GRAPH;
				}

				DataframeGraph concatGraph = new DataframeGraph(preroot);

				DataframeOperation concatNode = new Concat(pp.getLocation(), Concat.Axis.CONCAT_COLS);
				concatGraph.addNode(concatNode);
				concatGraph.addNode(exit1);
				concatGraph.addNode(exit2);
				concatGraph.addEdge(new SimpleEdge(preroot.getLeaf(), exit1));
				concatGraph.addEdge(new SimpleEdge(preroot.getLeaf(), exit2));
				concatGraph.addEdge(new ConcatEdge(exit1, concatNode, 1));
				concatGraph.addEdge(new ConcatEdge(exit2, concatNode, 0));

				return new DFOrConstant(new DataframeGraphDomain(concatGraph));
			} else {
				DataframeOperation exit1 = null;
				DataframeOperation exit2 = null;
				DataframeGraph df1graph = df1.getTransformations();
				DataframeGraph df2graph = df2.getTransformations();

				try {
					exit1 = df1graph.getLeaf();
					if (exit1 instanceof AccessOperation<?>) {
						DataframeOperation tmp = new ProjectionOperation(exit1.getWhere(),
								((AccessOperation<?>) exit1).getSelection());
						df1graph = df1graph.replaceNode(exit1, tmp);
						exit1 = tmp;
					}
					exit2 = df2graph.getLeaf();
					if (exit2 instanceof AccessOperation<?>) {
						DataframeOperation tmp = new ProjectionOperation(exit2.getWhere(),
								((AccessOperation<?>) exit2).getSelection());
						df2graph = df2graph.replaceNode(exit2, tmp);
						exit2 = tmp;
					}
				} catch (IllegalStateException e) {
					// at least one has more than one leaf
					return TOP_GRAPH;
				}

				DataframeGraph concatGraph = new DataframeGraph();
				concatGraph.mergeWith(df1graph);
				concatGraph.mergeWith(df2graph);

				DataframeOperation concatNode = new Concat(pp.getLocation(), Concat.Axis.CONCAT_COLS);
				concatGraph.addNode(concatNode);
				concatGraph.addEdge(new ConcatEdge(exit1, concatNode, 0));
				concatGraph.addEdge(new ConcatEdge(exit2, concatNode, 1));

				return new DFOrConstant(new DataframeGraphDomain(concatGraph));
			}
		} else if (operator instanceof WriteSelectionDataframe) {
			DataframeGraphDomain df1 = left.graph;
			DataframeGraphDomain df2 = right.graph;

			if (topOrBottom(df1) || topOrBottom(df2))
				return TOP_GRAPH;

			if (df1.equals(df2)) {
				// the last two nodes are the leaves to use for writing
				DataframeGraph original = df1.getTransformations();
				DataframeOperation righthand = original.getLeaf();
				if (righthand instanceof AccessOperation<?>)
					righthand = new ProjectionOperation(righthand.getWhere(),
							((AccessOperation<?>) righthand).getSelection());

				DataframeGraph prefix = original.prefix();
				DataframeOperation lefthand = prefix.getLeaf();
				if (!(lefthand instanceof SelectionOperation<?>))
					return TOP_GRAPH;
				if (lefthand instanceof AccessOperation<?>)
					lefthand = new ProjectionOperation(lefthand.getWhere(),
							((AccessOperation<?>) lefthand).getSelection());

				DataframeGraph result = new DataframeGraph(prefix.prefix());

				AssignDataframe assign = new AssignDataframe(pp.getLocation(),
						((SelectionOperation<?>) lefthand).getSelection());

				result.addNode(assign);
				result.addEdge(new SimpleEdge(lefthand, assign));
				result.addNode(righthand);
				result.addEdge(new AssignEdge(righthand, assign));

				DataframeGraphDomain dfNew = new DataframeGraphDomain(result);
				return new DFOrConstant(dfNew);
			} else {
				DataframeGraph original = df1.getTransformations();
				DataframeOperation access = original.getLeaf();
				if (!(access instanceof SelectionOperation<?>))
					return TOP_GRAPH;

				DataframeGraph prefix = original.prefix();

				DataframeGraph result = new DataframeGraph(prefix);
				DataframeGraph df2graph = df2.getTransformations();
				DataframeOperation df2leaf = df2graph.getLeaf();
				if (df2leaf instanceof AccessOperation<?>) {
					DataframeOperation tmp = new ProjectionOperation(df2leaf.getWhere(),
							((AccessOperation<?>) df2leaf).getSelection());
					df2graph = df2graph.replaceNode(df2leaf, tmp);
					df2leaf = tmp;
				}

				result.mergeWith(df2graph);

				AssignDataframe assign = new AssignDataframe(pp.getLocation(),
						((SelectionOperation<?>) access).getSelection());

				result.addNode(assign);
				result.addEdge(new SimpleEdge(prefix.getLeaf(), assign));
				result.addEdge(new AssignEdge(df2leaf, assign));

				DataframeGraphDomain dfNew = new DataframeGraphDomain(result);
				return new DFOrConstant(dfNew);
			}
		} else if (operator instanceof WriteSelectionConstant) {
			DataframeGraphDomain df = left.graph;
			if (topOrBottom(df))
				return TOP_GRAPH;

			DataframeOperation leaf = df.getTransformations().getLeaf();
			if (!(leaf instanceof SelectionOperation))
				return TOP_GRAPH;

			SelectionOperation<?> access = (SelectionOperation<?>) leaf;
			DataframeGraph resultGraph = df.getTransformations().prefix();

			// right can be either constant or df
			if (topOrBottom(right.constant))
				return TOP_GRAPH;

			DataframeSelection<?, ?> selection = (DataframeSelection<?, ?>) access.getSelection();
			DataframeOperation nodeToAdd = new AssignValue<>(pp.getLocation(), selection, right.constant);
			DataframeGraphDomain resultGraphDomain = new DataframeGraphDomain(resultGraph, nodeToAdd);
			return new DFOrConstant(resultGraphDomain);
		} else if (operator instanceof PandasSeriesComparison) {
			DataframeGraphDomain df1 = left.graph;
			ConstantPropagation value = right.constant;

			if (topOrBottom(df1) || topOrBottom(value))
				return TOP_GRAPH;

			DataframeGraph original = df1.getTransformations();
			DataframeOperation leaf = original.getLeaf();
			// we check if we had previously projected a part of the dataframe
			// which we want to compare
			if (!(leaf instanceof SelectionOperation<?>))
				return TOP_GRAPH;

			SelectionOperation<?> projection = (SelectionOperation<?>) leaf;

			// we remove the access node and replace with a comparison node
			DataframeGraph prefix = original.prefix();
			DataframeGraph result = new DataframeGraph(prefix);
			PandasSeriesComparison seriesCompOp = (PandasSeriesComparison) operator;

			if (!(projection.getSelection() instanceof ColumnListSelection))
				return TOP_GRAPH;

			AtomicBooleanSelection booleanSelection = new AtomicBooleanSelection(
					(ColumnListSelection) projection.getSelection(), seriesCompOp.getOp(), value);
			BooleanComparison<
					AtomicBooleanSelection> boolComp = new BooleanComparison<>(pp.getLocation(), booleanSelection);

			DataframeOperation prevLeaf = result.getLeaf();
			result.addNode(boolComp);
			result.addEdge(new SimpleEdge(prevLeaf, boolComp));

			return new DFOrConstant(new DataframeGraphDomain(result));
		} else
			return TOP;
	}

	@SuppressWarnings({ "unchecked", "rawtypes" })
	@Override
	protected DFOrConstant evalTernaryExpression(TernaryOperator operator, DFOrConstant left, DFOrConstant middle,
			DFOrConstant right, ProgramPoint pp) throws SemanticException {
		if (operator == DictPut.INSTANCE) {
			ConstantPropagation dict = left.constant;
			if (topOrBottom(dict) || topOrBottom(middle) || topOrBottom(right) || !dict.is(Map.class))
				return TOP_CONSTANT;

			ConstantPropagation newdict = new ConstantPropagation(
					new DictConstant(pp.getLocation(), dict.as(Map.class), Pair.of(middle, right)));

			return new DFOrConstant(newdict);
		} else if (operator == ProjectRows.INSTANCE || operator == AccessRows.INSTANCE) {
			DataframeGraphDomain df = left.graph;
			ConstantPropagation start = middle.constant;
			ConstantPropagation end = right.constant;

			if (topOrBottom(df) || topOrBottom(start) || topOrBottom(end))
				return TOP_GRAPH;

			NumberSlice slice = new NumberSlice(start.as(Integer.class), end.as(Integer.class));
			DataframeGraphDomain pr = new DataframeGraphDomain(df,
					operator == ProjectRows.INSTANCE ? new ProjectionOperation<>(pp.getLocation(), slice)
							: new AccessOperation<>(pp.getLocation(), slice));

			return new DFOrConstant(pr);
		} else if (operator instanceof AccessRowsColumns) {
			// left[middle, right]
			// df[row_slice | column_comparison, columns]
			DataframeGraphDomain df = left.graph;
			// right is a list of strings so we will handle that first
			ConstantPropagation cols = right.constant;
			if (topOrBottom(df) || topOrBottom(middle) || topOrBottom(cols) || !cols.is(List.class))
				return TOP_GRAPH;

			DataframeGraph resultGraph = df.getTransformations();
			DataframeSelection selection = new DataframeSelection(true);

			List<DFOrConstant> cs = cols.as(List.class);
			Set<String> accessedCols = new HashSet<>();
			ColumnListSelection colsSelection = null;

			for (DFOrConstant c : cs) {
				if (topOrBottom(c.constant) || !c.constant.is(String.class)) {
					colsSelection = new ColumnListSelection(true);
					break;
				}
				accessedCols.add(c.constant.as(String.class));
			}

			if (colsSelection == null)
				colsSelection = new ColumnListSelection(accessedCols);

			// middle can be either a slice or a series comparison
			if (!topOrBottom(middle.constant)) {
				// middle is a slice
				SliceConstant.Slice rowSlice = middle.constant.as(SliceConstant.Slice.class);

				NumberSlice numberSlice = new NumberSlice(
						rowSlice.getStart() == null ? new Interval().bottom() : rowSlice.getStart().toInterval(),
						rowSlice.getEnd() == null ? new Interval().bottom() : rowSlice.getEnd().toInterval(),
						rowSlice.getSkip() == null ? new Interval().bottom() : rowSlice.getSkip().toInterval());

				selection = new DataframeSelection(numberSlice, colsSelection);
			} else if (!topOrBottom(middle.graph)) {
				DataframeGraph middleGraph = middle.graph.getTransformations();
				if (!middleGraph.prefix().equals(df.getTransformations().prefix()))
					// df.loc[df["col"] < 5, ["col2", "col3"]]
					// we want to check we are selecting with cols of the same
					// dataframe
					return TOP_GRAPH;
				resultGraph = middleGraph.prefix();

				DataframeOperation leaf = middleGraph.getLeaf();
				if (!(leaf instanceof BooleanComparison))
					return TOP_GRAPH;
				BooleanComparison<?> colCompare = (BooleanComparison<?>) leaf;

				resultGraph = middleGraph.prefix();
				selection = new DataframeSelection(colCompare.getSelection(), colsSelection);
			}

			DataframeOperation access = new AccessOperation<>(pp.getLocation(), selection);
			return new DFOrConstant(new DataframeGraphDomain(resultGraph, access));
		} else if (operator instanceof SliceCreation) {
			if (left.constant.isBottom() || middle.constant.isBottom() || right.constant.isBottom()) {
				return TOP_CONSTANT;
			}

			RangeBound start = getRangeBound(left);
			RangeBound end = getRangeBound(middle);
			RangeBound skip = getRangeBound(right);
			return new DFOrConstant(new ConstantPropagation(new SliceConstant(start, end, skip, pp.getLocation())));
		} else
			return TOP;
	}

	private RangeBound getRangeBound(DFOrConstant c) {
		RangeBound bound;
		if (c.constant.isTop())
			bound = null;
		else if (c.constant.is(Integer.class))
			bound = new RangeBound(c.constant.as(Integer.class));
		else
			bound = c.constant.as(RangeBound.class);
		return bound;
	}
}
