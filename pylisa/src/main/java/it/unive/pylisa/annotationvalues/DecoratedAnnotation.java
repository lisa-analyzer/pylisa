package it.unive.pylisa.annotationvalues;


import it.unive.lisa.program.annotations.values.AnnotationValue;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.pylisa.antlr.Python3Parser;

import java.util.List;

public class DecoratedAnnotation implements AnnotationValue {
    private List<Expression> params;
    private Call call;

    public DecoratedAnnotation(List<Expression> params) {
        this.params = params;
        this.call = call;
    }

    public List<Expression> getParams() {
        return params;
    }

    public Call getCall() {
        return this.call;
    }

    public List<Expression> getParams() {
        return params;
    }

    @Override
    public int compareTo(AnnotationValue o) {
        return 0;
    }
}
