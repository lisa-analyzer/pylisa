package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.resolution.FixedOrderMatchingStrategy;

// TODO remove this
public class PythonLikeMatchingStrategy extends it.unive.lisa.program.cfg.statement.call.resolution.PythonLikeMatchingStrategy {

	public PythonLikeMatchingStrategy(FixedOrderMatchingStrategy delegate) {
		super(delegate);
	}

	@Override
	public boolean matches(Call call, Parameter[] formals, Expression[] actuals) {
		if (formals.length < actuals.length)
			return false;
		return super.matches(call, formals, actuals);
	}
}
