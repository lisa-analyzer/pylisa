package it.unive.pylisa.libraries.experimental.nativecfg.endpoints;

import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;

public abstract class GenericEndpoint extends NativeCFG {
    private Class<? extends NaryExpression> construct;
    /**
     * Builds the native control flow graph.
     *
     * @param descriptor the descriptor of this cfg
     * @param construct  the class of the {@link NaryExpression} that provides
     *                   the semantics of this native cfg; the class of the
     *                   construct must also be a subtype of
     *                   {@link PluggableStatement}
     * @throws IllegalArgumentException if the class of the construct does not
     *                                  implement {@link PluggableStatement}
     */
    public GenericEndpoint(CodeMemberDescriptor descriptor, Class<? extends NaryExpression> construct) {
        super(descriptor, construct);
        this.construct = construct;
    }

    public Class<? extends NaryExpression> getConstruct() {
        return this.construct;
    }
}
