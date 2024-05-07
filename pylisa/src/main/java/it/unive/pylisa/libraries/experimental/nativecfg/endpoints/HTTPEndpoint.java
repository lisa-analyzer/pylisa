package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.libraries.experimental.statement.HTTPEndpointStatement;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

public class HTTPEndpoint extends GenericEndpoint {


    Mapping method;

    Mapping path;


    /**
     * Builds the native control flow graph.
     *
     * @param descriptor the descriptor of this cfg
     * @throws IllegalArgumentException if the class of the construct does not
     *                                  implement {@link PluggableStatement}
     */
    public HTTPEndpoint(CodeMemberDescriptor descriptor, Mapping method, Mapping path) {
        super(descriptor, HTTPEndpointStatement.class);
        this.method = method;
        this.path = path;
    }

    @Override
    public NaryExpression rewrite(Statement original, Expression... params) throws CallResolutionException {
        try {
            Expression[] parameters = new Expression[2];
            parameters[0] = method.eval(original);
            parameters[1] = path.eval(original);
            Method builder = getConstruct().getDeclaredMethod("build", CFG.class, CodeLocation.class, Expression[].class);
            NaryExpression instance = (NaryExpression) builder.invoke(null, original.getCFG(), original.getLocation(),
                    parameters);
            ((PluggableStatement) instance).setOriginatingStatement(original);
            return instance;
        } catch (NoSuchMethodException
             | SecurityException
             | IllegalAccessException
             | IllegalArgumentException
             | InvocationTargetException e) {
            throw new CallResolutionException("Unable to create call to native construct " + getConstruct().getName(), e);
        }
    }
}