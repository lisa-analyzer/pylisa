package it.unive.pylisa.symbolic;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import java.util.Arrays;
import java.util.List;

public class LambdaConstant extends Constant {

	private final List<Expression> arguments;

	public LambdaConstant(Type type, CodeLocation location, List<Expression> arguments, Expression body) {
		super(type, body, location);
		this.arguments = arguments;
	}

	public List<Expression> getArguments() {
		return arguments;
	}

	@Override
	public Expression getValue() {
		return (Expression) super.getValue();
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((arguments == null) ? 0 : arguments.hashCode());
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		LambdaConstant other = (LambdaConstant) obj;
		if (arguments == null) {
			if (other.arguments != null)
				return false;
		} else if (!arguments.equals(other.arguments))
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "lambda " + Arrays.toString(arguments.toArray()) + " : " + getValue();
	}
}
