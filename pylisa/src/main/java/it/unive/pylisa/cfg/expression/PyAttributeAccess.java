package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.annotations.Annotations;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.UnaryExpression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import java.util.HashSet;
import java.util.Set;

/**
 * An access to the attribute {@code name} of an object ({@code receiver.name}),
 * both when reading it and as the target of an assignment
 * ({@code receiver.name = v}). When the receiver is an object, the attribute is
 * the field {@code name} of the object in the heap, so that its value is
 * tracked across reads and writes (also through aliases, and inside called
 * functions). For any other receiver (e.g. a {@code str}, whose attributes are
 * its methods), the value is unknown.
 */
public class PyAttributeAccess extends UnaryExpression {

	private final String name;

	/**
	 * Builds the access.
	 *
	 * @param cfg      the cfg where the access happens
	 * @param location the location of the access
	 * @param receiver the object whose attribute is accessed
	 * @param name     the name of the attribute
	 */
	public PyAttributeAccess(
			CFG cfg,
			CodeLocation location,
			Expression receiver,
			String name) {
		super(cfg, location, ".", receiver);
		this.name = name;
	}

	/**
	 * Yields the name of the accessed attribute.
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return name.compareTo(((PyAttributeAccess) o).name);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdUnarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression expr,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		CodeLocation loc = getLocation();

		Set<Type> objects = new HashSet<>();
		boolean others = false;
		Set<Type> types = analysis.getRuntimeTypesOf(state, expr, this);
		if (types.isEmpty())
			others = true;
		for (Type t : types)
			if (t.isPointerType() && t.asPointerType().getInnerType().isUnitType())
				objects.add(t.asPointerType().getInnerType());
			else
				others = true;

		AnalysisState<A> result = state.bottom();
		if (!objects.isEmpty()) {
			// python objects have no declared fields: the attribute is a field
			// created by its first assignment
			HeapDereference container = new HeapDereference(Type.commonSupertype(objects, Untyped.INSTANCE), expr,
					loc);
			Variable field = new Variable(Untyped.INSTANCE, name, new Annotations(), loc);
			result = result.lub(
					analysis.smallStepSemantics(state, new AccessChild(Untyped.INSTANCE, container, field, loc), this));
		}
		if (others)
			result = result.lub(analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), this));
		return result;
	}

	@Override
	public String toString() {
		return getSubExpression() + "." + name;
	}

	@Override
	public int hashCode() {
		return 31 * super.hashCode() + name.hashCode();
	}

	@Override
	public boolean equals(
			Object obj) {
		return super.equals(obj) && name.equals(((PyAttributeAccess) obj).name);
	}
}
