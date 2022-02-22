package it.unive.pylisa.analysis.dataframes;

import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
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
import it.unive.pylisa.analysis.dataframes.transformation.operations.ReadFromFile;
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
		if (constant.isBottom() != !other.constant.isBottom())
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

			DataframeGraphDomain df = new DataframeGraphDomain(new ReadFromFile(filename.as(String.class)));
			return new DFOrConstant(df);
//		} else if (operator == Statistics.INSTANCE) {
//			DataframeGraphDomain df = arg.graph;
//			DataframeGraphDomain stat = new DataframeGraphDomain(df, new BaseOperation("stats"));
//			return new DFOrConstant(stat);
//		} else if (operator == StructuralInfo.INSTANCE) {
//			DataframeGraphDomain df = arg.graph;
//			DataframeGraphDomain info = new DataframeGraphDomain(df, new BaseOperation("info"));
//			return new DFOrConstant(info);
//		} else if (operator instanceof TypeConversion) {
//			DataframeGraphDomain df = arg.graph;
//			DataframeGraphDomain conv = new DataframeGraphDomain(df,
//					new BaseOperation("conv", ((TypeConversion) operator).getType()));
//			return new DFOrConstant(conv);
		} else
			return TOP;
	}

	@Override
	protected DFOrConstant evalBinaryExpression(BinaryOperator operator, DFOrConstant left, DFOrConstant right,
			ProgramPoint pp) throws SemanticException {
//		if (operator == ColumnAccess.INSTANCE) {
//			DataframeGraphDomain df = left.graph;
//			ConstantPropagation col = right.constant;
//			if (topOrBottom(df) || topOrBottom(col))
//				return new DFOrConstant(graph.top());
//
//			DataframeGraphDomain ca = new DataframeGraphDomain(df,
//					new BaseOperation("col_access", col.as(String.class)));
//
//			return new DFOrConstant(ca);
//		} else
			return TOP;
	}

	@Override
	protected DFOrConstant evalTernaryExpression(TernaryOperator operator, DFOrConstant left, DFOrConstant middle,
			DFOrConstant right, ProgramPoint pp) throws SemanticException {
//		if (operator == ProjectRows.INSTANCE) {
//			DataframeGraphDomain df = left.graph;
//			ConstantPropagation start = middle.constant;
//			ConstantPropagation end = right.constant;
//
//			if (topOrBottom(df) || topOrBottom(start) || topOrBottom(end))
//				return new DFOrConstant(left.top());
//
//			DataframeGraphDomain pr = new DataframeGraphDomain(df,
//					new BaseOperation("project_rows", start.as(Integer.class),
//							end.as(Integer.class)));
//
//			return new DFOrConstant(pr);
//		} else if (operator == SetOptionAux.INSTANCE) {
//			DataframeGraphDomain df = left.graph;
//			ConstantPropagation key = middle.constant;
//			ConstantPropagation value = right.constant;
//
//			if (topOrBottom(df) || topOrBottom(key) || topOrBottom(value))
//				return new DFOrConstant(left.top());
//
//			DataframeGraphDomain pr = new DataframeGraphDomain(df,
//					new BaseOperation("set_opt", key.as(String.class),
//							value.getConstant()));
//
//			return new DFOrConstant(pr);
//		} else
			return TOP;
	}
}
