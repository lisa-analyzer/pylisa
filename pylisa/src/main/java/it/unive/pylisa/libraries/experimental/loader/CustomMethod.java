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
         * FastAPI.get(
         *     self libtype fastapi.FastAPI*
         *     path type it.unive.lisa.program.type.StringType::INSTANCE
         *     &response_model type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &status_code type it.unive.lisa.program.type.Int32Type::INSTANCE default 0
         *     &tags type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &dependencies type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &summary type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &description type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &response_description type it.unive.lisa.program.type.StringType::INSTANCE default "Successful Response"
         *     &responses type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &deprecated type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &operation_id type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &response_model_include type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &response_model_exclude type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &response_model_by_alias type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *     &response_model_exclude_unset type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &response_model_exclude_defaults type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &response_model_exclude_none type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &include_in_schema type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *     &response_class type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &name type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &callbacks type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &openapi_extra type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &generate_unique_id_function type it.unive.lisa.type.Untyped::INSTANCE default none
         * ) -> HTTPEndpoint(
         *     path: $param[1] IF instanceof($param[1], NamedParameterExpression) ELSE $param["path"]
         *     method: String("GET")
         * )
         *
         * FastAPI.post(
         *     self libtype fastapi.FastAPI*
         *     path type it.unive.lisa.program.type.StringType::INSTANCE
         *     &response_model type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &status_code type it.unive.lisa.program.type.Int32Type::INSTANCE default 0
         *     &tags type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &dependencies type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &summary type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &description type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &response_description type it.unive.lisa.program.type.StringType::INSTANCE default "Successful Response"
         *     &responses type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &deprecated type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &operation_id type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &response_model_include type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &response_model_exclude type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &response_model_by_alias type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *     &response_model_exclude_unset type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &response_model_exclude_defaults type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &response_model_exclude_none type it.unive.lisa.program.type.BoolType::INSTANCE default false
         *     &include_in_schema type it.unive.lisa.program.type.BoolType::INSTANCE default true
         *     &response_class type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &name type it.unive.lisa.program.type.StringType::INSTANCE default ""
         *     &callbacks type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &openapi_extra type it.unive.lisa.type.Untyped::INSTANCE default none
         *     &generate_unique_id_function type it.unive.lisa.type.Untyped::INSTANCE default none
         * ) -> HTTPEndpoint(
         *     path: $param[1] IF instanceof($param[1], NamedParameterExpression) ELSE $param["path"]
         *     method: String("GET")
         * )
         */

        // es per GET



        /*
        get(
            self,
            path,
            *,
            response_model=Default(None),
            status_code=None,
            tags=None,
            dependencies=None,
            summary=None,
            description=None,
            response_description="Successful Response",
            responses=None,
            deprecated=None,
            operation_id=None,
            response_model_include=None,
            response_model_exclude=None,
            response_model_by_alias=True,
            response_model_exclude_unset=False,
            response_model_exclude_defaults=False,
            response_model_exclude_none=False,
            include_in_schema=True,
            response_class=Default(JSONResponse),
            name=None,
            callbacks=None,
            openapi_extra=None,
            generate_unique_id_function=Default(generate_unique_id)
        )

        @app.get("/status")
        @app.get(path = "/status")
        @app.get(status_code = 200, path = "/status")



        def f(a, b, *, x, y):
            print(a)
        f(10,20, x=30,y=40)
        f(10,20,y=40,x=30)
        f(10,x=30,b=20,y=40)
        f(x=30,y=40,b=20,a=10)

         */
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
