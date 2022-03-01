package it.unive.pylisa.analysis.dataframes;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.lattices.ExpressionSet;
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
import it.unive.pylisa.analysis.dataframes.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.transformation.DataframeGraphDomain;
import it.unive.pylisa.analysis.dataframes.transformation.Names;
import it.unive.pylisa.analysis.dataframes.transformation.graph.DataframeGraph;
import it.unive.pylisa.analysis.dataframes.transformation.graph.SimpleEdge;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ColAccess;
import it.unive.pylisa.analysis.dataframes.transformation.operations.Concat;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DataframeOperation;
import it.unive.pylisa.analysis.dataframes.transformation.operations.DropColumns;
import it.unive.pylisa.analysis.dataframes.transformation.operations.FilterNullRows;
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
import it.unive.pylisa.analysis.dataframes.transformation.operations.RowAccess;
import it.unive.pylisa.analysis.dataframes.transformation.operations.RowProjection;
import it.unive.pylisa.symbolic.operators.AccessRows;
import it.unive.pylisa.symbolic.operators.ColumnAccess;
import it.unive.pylisa.symbolic.operators.ConcatCols;
import it.unive.pylisa.symbolic.operators.ConcatRows;
import it.unive.pylisa.symbolic.operators.Drop;
import it.unive.pylisa.symbolic.operators.FilterNull;
import it.unive.pylisa.symbolic.operators.ProjectRows;
import it.unive.pylisa.symbolic.operators.ReadDataframe;

public class DFOrConstant extends BaseNonRelationalValueDomain<DFOrConstant> {

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
			return Lattice.TOP_REPR;
		if (isBottom())
			return Lattice.BOTTOM_REPR;
		if (graph == null)
			return constant.representation();
		return graph.representation();
	}

	@Override
	public boolean tracksIdentifiers(Identifier id) {
		return ConstantPropagation.tracks(id) || DataframeGraphDomain.tracks(id);
	}

	@Override
	public boolean canProcess(SymbolicExpression expression) {
		return ConstantPropagation.processes(expression) || DataframeGraphDomain.processes(expression);
	}

	private static boolean topOrBottom(Lattice<?> l) {
		return l.isTop() || l.isBottom();
	}

	@Override
	protected DFOrConstant evalIdentifier(Identifier id, ValueEnvironment<DFOrConstant> environment, ProgramPoint pp)
			throws SemanticException {
		if (id instanceof MemoryPointer)
			return environment.getState(((MemoryPointer) id).getReferencedLocation());
		return super.evalIdentifier(id, environment, pp);
	}

	@Override
	protected DFOrConstant evalNonNullConstant(Constant constant, ProgramPoint pp) throws SemanticException {
		return new DFOrConstant(this.constant.eval(constant));
	}

	@Override
	protected DFOrConstant evalUnaryExpression(UnaryOperator operator, DFOrConstant arg, ProgramPoint pp)
			throws SemanticException {
		if (operator == ReadDataframe.INSTANCE) {
			ConstantPropagation filename = arg.constant;
			if (topOrBottom(filename))
				return new DFOrConstant(graph.top());

			DataframeGraphDomain df = new DataframeGraphDomain(
					new ReadFromFile(pp.getLocation(), filename.as(String.class)));
			return new DFOrConstant(df);
//		} else if (operator instanceof TypeConversion) {
//			DataframeGraphDomain df = arg.graph;
//			DataframeGraphDomain conv = new DataframeGraphDomain(df,
//					new BaseOperation("conv", ((TypeConversion) operator).getType()));
//			return new DFOrConstant(conv);
		} else if (operator == FilterNull.INSTANCE) {
			DataframeGraphDomain df = arg.graph;
			if (topOrBottom(df))
				return new DFOrConstant(graph.top());

			DataframeGraphDomain dfNew = new DataframeGraphDomain(df, new FilterNullRows(pp.getLocation()));
			return new DFOrConstant(dfNew);
		} else
			return TOP;
	}

	@Override
	protected DFOrConstant evalBinaryExpression(BinaryOperator operator, DFOrConstant left, DFOrConstant right,
			ProgramPoint pp) throws SemanticException {
		if (operator == ColumnAccess.INSTANCE) {
			DataframeGraphDomain df = left.graph;
			ConstantPropagation col = right.constant;
			if (topOrBottom(df) || topOrBottom(col))
				return new DFOrConstant(graph.top());

			DataframeGraphDomain ca = new DataframeGraphDomain(df,
					new ColAccess(pp.getLocation(), new Names(col.as(String.class))));

			return new DFOrConstant(ca);
		} else if (operator == Drop.INSTANCE) {
			DataframeGraphDomain df = left.graph;
			ConstantPropagation cols = right.constant;
			if (topOrBottom(df) || topOrBottom(cols))
				return new DFOrConstant(graph.top());

			if (!(cols.getConstant() instanceof ExpressionSet<?>[]))
				// check whether the cols constant is indeed what we expect for a constant list
				return new DFOrConstant(graph.top());

			ExpressionSet<Constant>[] cs = (ExpressionSet<Constant>[]) cols.getConstant();
			Set<String> accessedCols = new HashSet<>();

			for (ExpressionSet<Constant> c : cs) {
				for (Constant colName : c) {
					if (!(colName.getValue() instanceof String))
						return new DFOrConstant(graph.top());
					accessedCols.add((String) colName.getValue());
				}
			}

			DataframeGraphDomain ca = new DataframeGraphDomain(df, new DropColumns(accessedCols));

			return new DFOrConstant(ca);
		} else if (operator == ConcatCols.INSTANCE || operator == ConcatRows.INSTANCE) {
			DataframeGraphDomain df1 = left.graph;
			DataframeGraphDomain df2 = right.graph;

			if (df1.isBottom() || df2.isBottom())
				return new DFOrConstant(graph.top());
			
			if (df1.isTop() || df2.isTop())
				return new DFOrConstant(graph.top());

			Collection<DataframeOperation> exit1c = df1.getTransformations().getAdjacencyMatrix().getExits();
			Collection<DataframeOperation> exit2c = df2.getTransformations().getAdjacencyMatrix().getExits();

			if (exit1c.size() != 1 || exit2c.size() != 1) {
				return new DFOrConstant(graph.top());
			}

			DataframeOperation exit1 = exit1c.iterator().next();
			DataframeOperation exit2 = exit2c.iterator().next();

			DataframeGraph concatGraph = new DataframeGraph();
			concatGraph.getAdjacencyMatrix().mergeWith(df1.getTransformations().getAdjacencyMatrix());
			concatGraph.getAdjacencyMatrix().mergeWith(df2.getTransformations().getAdjacencyMatrix());

			DataframeOperation concatNode = new Concat(operator == ConcatCols.INSTANCE ? Concat.Axis.CONCAT_COLS : Concat.Axis.CONCAT_ROWS);
			concatGraph.addNode(concatNode);
			concatGraph.addEdge(new SimpleEdge(exit1, concatNode, 0));
			concatGraph.addEdge(new SimpleEdge(exit2, concatNode, 1));

			return new DFOrConstant(new DataframeGraphDomain(concatGraph));
		} else
			return TOP;
	}

	@Override
	protected DFOrConstant evalTernaryExpression(TernaryOperator operator, DFOrConstant left, DFOrConstant middle,
			DFOrConstant right, ProgramPoint pp) throws SemanticException {
		if (operator == ProjectRows.INSTANCE || operator == AccessRows.INSTANCE) {
			DataframeGraphDomain df = left.graph;
			ConstantPropagation start = middle.constant;
			ConstantPropagation end = right.constant;

			if (topOrBottom(df) || topOrBottom(start) || topOrBottom(end))
				return new DFOrConstant(graph.top());

			Interval rows = new Interval(start.as(Integer.class), end.as(Integer.class));
			DataframeGraphDomain pr = new DataframeGraphDomain(df,
					operator == ProjectRows.INSTANCE ? new RowProjection(pp.getLocation(), rows)
							: new RowAccess(pp.getLocation(), rows));

			return new DFOrConstant(pr);
		} else
			return TOP;
	}
}
