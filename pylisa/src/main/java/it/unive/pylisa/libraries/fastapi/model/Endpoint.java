package it.unive.pylisa.libraries.fastapi.model;

import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.pylisa.annotationvalues.DecoratedAnnotation;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.expression.PyStringLiteral;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
public class Endpoint {

    private Method method;
    private String fullPath;
    private String pathVariableName;
    private List<Param> methodPathVariable = new ArrayList<>();
    private Boolean calledAtleastOnce = false;

    public Endpoint(AnnotationMember decorator) {

        this.method = Method.specify(decorator.getId());

        DecoratedAnnotation decoratorValue = (DecoratedAnnotation) decorator.getValue();
        for (Expression exp : decoratorValue.getParams()) {

            if (exp instanceof PyAssign pyAssign) {

                List<Expression> subExpressions = Arrays.stream(pyAssign.getSubExpressions()).toList();

                VariableRef argument = (VariableRef) subExpressions.get(0);
                PyStringLiteral path = (PyStringLiteral) subExpressions.get(1);

                if (argument.getName().equals("path")) {
                    this.fullPath = path.getValue();
                    this.pathVariableName = extractPathVariable(this.fullPath);
                }
            }
        }
    }

    public static String extractPathVariable(String fullPath) {

        if (fullPath == null) return null;

        String pattern = "/([^/]+)/\\{([^}]+)\\}";
        Pattern r = Pattern.compile(pattern);
        Matcher m = r.matcher(fullPath);

        String pathVariable = "";
        while (m.find()) {
            pathVariable = m.group(2);
        }

        return pathVariable;
    }
}
