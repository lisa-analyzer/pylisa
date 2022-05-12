package it.unive.pylisa.analysis.string;

import it.unive.lisa.analysis.BaseLattice;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticDomain.Satisfiability;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.nonrelational.value.BaseNonRelationalValueDomain;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.representation.DomainRepresentation;
import it.unive.lisa.analysis.representation.StringRepresentation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.AdditionOperator;
import it.unive.lisa.symbolic.value.operator.DivisionOperator;
import it.unive.lisa.symbolic.value.operator.Module;
import it.unive.lisa.symbolic.value.operator.Multiplication;
import it.unive.lisa.symbolic.value.operator.SubtractionOperator;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonNe;
import it.unive.lisa.symbolic.value.operator.ternary.TernaryOperator;
import it.unive.lisa.symbolic.value.operator.unary.NumericNegation;
import it.unive.lisa.symbolic.value.operator.unary.UnaryOperator;

/**
 * The overflow-insensitive basic integer constant propagation abstract domain,
 * tracking if a certain integer value has constant value or not, implemented as
 * a {@link BaseNonRelationalValueDomain}, handling top and bottom values for
 * the expression evaluation and bottom values for the expression
 * satisfiability. Top and bottom cases for least upper bounds, widening and
 * less or equals operations are handled by {@link BaseLattice} in
 * {@link BaseLattice#lub}, {@link BaseLattice#widening} and
 * {@link BaseLattice#lessOrEqual}, respectively.
 *
 * @author <a href="mailto:vincenzo.arceri@unive.it">Vincenzo Arceri</a>
 */
public class StringConstantPropagation extends BaseNonRelationalValueDomain<StringConstantPropagation> {

    private static final StringConstantPropagation TOP = new StringConstantPropagation(true, false);
    private static final StringConstantPropagation BOTTOM = new StringConstantPropagation(false, true);

    private final boolean isTop, isBottom;

    private final String value;

    /**
     * Builds the top abstract value.
     */
    public StringConstantPropagation() {
        this(null, true, false);
    }

    private StringConstantPropagation(String value, boolean isTop, boolean isBottom) {
        this.value = value;
        this.isTop = isTop;
        this.isBottom = isBottom;
    }

    private StringConstantPropagation(String value) {
        this(value, false, false);
    }

    private StringConstantPropagation(boolean isTop, boolean isBottom) {
        this(null, isTop, isBottom);
    }

    @Override
    public StringConstantPropagation top() {
        return TOP;
    }

    @Override
    public boolean isTop() {
        return isTop;
    }

    @Override
    public StringConstantPropagation bottom() {
        return BOTTOM;
    }

    @Override
    public DomainRepresentation representation() {
        if (isBottom())
            return Lattice.BOTTOM_REPR;
        if (isTop())
            return Lattice.TOP_REPR;

        return new StringRepresentation(value.toString());
    }

    @Override
    protected StringConstantPropagation evalNullConstant(ProgramPoint pp) {
        return top();
    }

    @Override
    protected StringConstantPropagation evalNonNullConstant(Constant constant, ProgramPoint pp) {
        if (constant.getValue() instanceof String)
            return new StringConstantPropagation((String) constant.getValue());
        return top();
    }

    @Override
    protected StringConstantPropagation evalUnaryExpression(UnaryOperator operator, StringConstantPropagation arg,
                                                             ProgramPoint pp) {
        return top();
    }

    @Override
    protected StringConstantPropagation evalBinaryExpression(BinaryOperator operator, StringConstantPropagation left,
                                                              StringConstantPropagation right, ProgramPoint pp) {
            return top();
    }

    @Override
    protected StringConstantPropagation evalTernaryExpression(TernaryOperator operator,
                                                               StringConstantPropagation left,
                                                               StringConstantPropagation middle, StringConstantPropagation right, ProgramPoint pp) {
        return top();
    }

    @Override
    protected StringConstantPropagation lubAux(StringConstantPropagation other) throws SemanticException {
        return TOP;
    }

    @Override
    protected StringConstantPropagation wideningAux(StringConstantPropagation other) throws SemanticException {
        return lubAux(other);
    }

    @Override
    protected boolean lessOrEqualAux(StringConstantPropagation other) throws SemanticException {
        return false;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + (isBottom ? 1231 : 1237);
        result = prime * result + (isTop ? 1231 : 1237);
        result = prime * result + ((value == null) ? 0 : value.hashCode());
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        StringConstantPropagation other = (StringConstantPropagation) obj;
        if (isBottom != other.isBottom)
            return false;
        if (isTop != other.isTop)
            return false;
        if (value == null) {
            if (other.value != null)
                return false;
        } else if (!value.equals(other.value))
            return false;
        return true;
    }

    @Override
    protected Satisfiability satisfiesBinaryExpression(BinaryOperator operator, StringConstantPropagation left,
                                                       StringConstantPropagation right, ProgramPoint pp) {
            return Satisfiability.UNKNOWN;
    }

    @Override
    protected ValueEnvironment<StringConstantPropagation> assumeBinaryExpression(
            ValueEnvironment<StringConstantPropagation> environment, BinaryOperator operator, ValueExpression left,
            ValueExpression right, ProgramPoint pp) throws SemanticException {
        return environment;
    }
}