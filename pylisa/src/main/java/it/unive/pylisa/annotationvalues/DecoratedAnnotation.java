package it.unive.pylisa.annotationvalues;


import it.unive.lisa.program.annotations.values.AnnotationValue;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.antlr.Python3Parser;

import java.util.List;

public class DecoratedAnnotation implements AnnotationValue {
    private List<Expression> params;

    public DecoratedAnnotation(List<Expression> params) {
        this.params = params;
    }

    @Override
    public int compareTo(AnnotationValue o) {
        return 0;
    }
}
