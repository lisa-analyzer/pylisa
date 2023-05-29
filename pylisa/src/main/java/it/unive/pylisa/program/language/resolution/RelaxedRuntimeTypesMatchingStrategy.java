package it.unive.pylisa.program.language.resolution;

import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.language.resolution.FixedOrderMatchingStrategy;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;

import java.util.Set;

public class RelaxedRuntimeTypesMatchingStrategy extends FixedOrderMatchingStrategy {
    public static final RelaxedRuntimeTypesMatchingStrategy INSTANCE = new RelaxedRuntimeTypesMatchingStrategy();


    @Override
    public boolean matches(Call call, int pos, Parameter formal, Expression actual, Set<Type> types) {
        return formal.getStaticType() instanceof Untyped || types.stream().anyMatch(rt -> rt.canBeAssignedTo(formal.getStaticType()));
    }
}
