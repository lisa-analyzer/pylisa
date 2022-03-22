package it.unive.pylisa.symbolic;

import java.util.List;

import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.pylisa.cfg.type.PyListType;

public class AtomList extends Constant {

	public AtomList(CodeLocation location, List<ExpressionSet<ValueExpression>> constants) {
		super(PyListType.INSTANCE, constants, location);
		for (ExpressionSet<ValueExpression> constant : constants)
			for (ValueExpression c : constant)
				if (!isAtom(c))
					throw new IllegalArgumentException("Unsupported constant type: " + c.getClass().getName());
	}

	@SuppressWarnings("unchecked")
	public List<ExpressionSet<ValueExpression>> getList() {
		return (List<ExpressionSet<ValueExpression>>) getValue();
	}

	public static boolean isAtom(SymbolicExpression expr) {
		return expr instanceof Constant || expr instanceof Identifier;
	}
}
