package it.unive.pylisa.libraries.experimental.loader;

import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.pylisa.libraries.LibrarySpecificationParser;
import it.unive.pylisa.libraries.experimental.nativecfg.endpoints.*;
import it.unive.pylisa.libraries.experimental.statement.HTTPEndpointStatement;
import it.unive.pylisa.libraries.loader.Method;
import it.unive.pylisa.libraries.loader.Parameter;
import it.unive.pylisa.libraries.loader.Type;

public class CustomMethod extends Method {
    public CustomMethod(boolean instance, boolean sealed, String name, String implementation, Type type) {
        super(instance, sealed, name, implementation, type);
    }

    @Override
    public NativeCFG toLiSACfg(
            CodeLocation location,
            CFG init,
            Unit container) {
        it.unive.lisa.program.cfg.Parameter[] pars = new it.unive.lisa.program.cfg.Parameter[this.getParams().size()];
        int i = 0;
        for (Parameter p : this.getParams()) {
            pars[i] = p.toLiSAParameter(location, init);
            i++;
        }
        CodeMemberDescriptor desc = new CodeMemberDescriptor(
                location,
                container,
                this.isInstance(),
                this.getName(),
                this.getType().toLiSAType(),
                pars);

        desc.setOverridable(this.isSealed());

        /**
         * FastAPI.get -> HTTPEndpoint {
         *             libtype fastapi.FastAPI*
         *             param self libtype fastapi.FastAPI*
         *             param &debug type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *             param &routes type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &title type it.unive.lisa.program.type.StringType::INSTANCE default "FastAPI"
         *             param &summary type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *             param &description type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *             param &version type it.unive.lisa.program.type.StringType::INSTANCE default "0.1.0"
         *             param &openapi_url type it.unive.lisa.program.type.StringType::INSTANCE default "/openapi.json"
         *             param &openapi_tags type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &servers type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &dependencies type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &default_response_class type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &redirect_slashes type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *             param &docs_url type it.unive.lisa.program.type.StringType::INSTANCE default "/docs"
         *             param &redoc_url type it.unive.lisa.program.type.StringType::INSTANCE default "/redoc"
         *             param &swagger_ui_oauth2_redirect_url type it.unive.lisa.program.type.StringType::INSTANCE default "/redoc"
         *             param &swagger_ui_init_oauth type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &middleware type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &exception_handlers type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &on_startup type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &on_shutdown type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &lifespan type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &terms_of_service type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &contact type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &license_info type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &openapi_prefix type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *             param &root_path type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *             param &root_path_in_servers type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *             param &responses type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &callbacks type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &webhooks type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &deprecated type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &include_in_schema type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *             param &swagger_ui_parameters type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &generate_unique_id_function type it.unive.lisa.type.Untyped::INSTANCE default none
         *             param &separate_input_output_schemas type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *             param **extra type it.unive.lisa.type.Untyped::INSTANCE default none
         *
         *      path: $param[1] IF instanceof($param[1], NamedParameterExpression) ELSE $param["path"]
         *      method: String("GET")
         * }
         *
         * FastAPI.post -> HTTPEndpoint {
         *      path: $param[1] IF instanceof($param[1], NamedParameterExpression) ELSE $param["path"]
         *      method: String("POST")
         * }
         */

        // es per GET
        return new HTTPEndpoint(desc,
                new Mapping("path",
                        new ConditionalMapping(new IsInstanceOf(new ParameterMappingRule(1), NamedParameterExpression.class), // param[1] instanceof NamedParameterExpression ?
                                new NamedParameterMappingRule("path"), // (return named param "path") [TRUE BRANCH]
                                new ParameterMappingRule(1) // param[1] [FALSE BRANCH]
                        )
                ),
                new Mapping("method", new StringConstantMappingRule("GET")));
    }
}
