package it.unive.pylisa.analysis.types;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.lattices.Satisfiability;
import it.unive.lisa.analysis.nonrelational.type.BaseNonRelationalTypeDomain;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.*;
import it.unive.lisa.symbolic.value.operator.binary.*;
import it.unive.lisa.symbolic.value.operator.unary.LogicalNegation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.lisa.type.TypeTokenType;

import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Collectors;

import static org.apache.commons.collections4.SetUtils.difference;
import static org.apache.commons.collections4.SetUtils.intersection;

public class PythonInferredTypes implements
        BaseNonRelationalTypeDomain<PythonTypeSet> {

    @Override
    public PythonTypeSet evalIdentifier(
            Identifier id,
            TypeEnvironment<PythonTypeSet> environment,
            ProgramPoint pp,
            SemanticOracle oracle)
            throws SemanticException {
        TypeSystem types = pp.getProgram().getTypes();
        if (id instanceof HeapLocation && ((HeapLocation) id).isAllocation())
            // if this is a heap location that is being allocated,
            // its types are exactly the static type
            return new PythonTypeSet(types, id.getStaticType());
        if (id instanceof MemoryPointer) {
            // if this is a memory pointer, its types are all the types
            // that can be pointed to by the static type of the pointer
            MemoryPointer mp = (MemoryPointer) id;
           PythonTypeSet inner = evalIdentifier(mp.getReferencedLocation(), environment, pp, oracle);
            if (inner.isTop())
                return new PythonTypeSet(types, id.getStaticType().allInstances(types));
            if (inner.isBottom())
                return PythonTypeSet.BOTTOM;
            TypeSystem ts = pp.getProgram().getTypes();
            return new PythonTypeSet(
                    types,
                    inner.elements.stream().map(t -> ts.getReference(t)).collect(Collectors.toSet()));
        }
       PythonTypeSet eval = BaseNonRelationalTypeDomain.super.evalIdentifier(id, environment, pp, oracle);

        if (!eval.isTop())
            return eval;
        if (eval.isTop() && !eval.isEmpty()) {
            return eval;
        }
        return new PythonTypeSet(types, id.getStaticType().allInstances(types));
    }

    @Override
    public PythonTypeSet evalPushAny(
            PushAny pushAny,
            ProgramPoint pp,
            SemanticOracle oracle)
            throws SemanticException {
        TypeSystem types = pp.getProgram().getTypes();
        if (pushAny.getStaticType().isUntyped())
            return top();
        return new PythonTypeSet(types, pushAny.getStaticType().allInstances(types));
    }

    @Override
    public PythonTypeSet evalPushInv(
            PushInv pushInv,
            ProgramPoint pp,
            SemanticOracle oracle)
            throws SemanticException {
        return bottom();
    }

    @Override
    public PythonTypeSet evalConstant(
            Constant constant,
            ProgramPoint pp,
            SemanticOracle oracle) {
        return new PythonTypeSet(pp.getProgram().getTypes(), constant.getStaticType());
    }

    @Override
    public PythonTypeSet evalUnaryExpression(
            UnaryExpression expression,
           PythonTypeSet arg,
            ProgramPoint pp,
            SemanticOracle oracle) {
        TypeSystem types = pp.getProgram().getTypes();
        Set<Type> elems = arg.isTop() ? types.getTypes() : arg.elements;
        Set<Type> inferred = expression.getOperator().typeInference(types, elems);
        if (inferred.isEmpty())
            return PythonTypeSet.BOTTOM;
        return new PythonTypeSet(types, inferred);
    }

    @Override
    public PythonTypeSet evalBinaryExpression(
            BinaryExpression expression,
            PythonTypeSet left,
            PythonTypeSet right,
            ProgramPoint pp,
            SemanticOracle oracle) {
        TypeSystem types = pp.getProgram().getTypes();
        Set<Type> lelems = left.isTop() ? types.getTypes() : left.elements;
        Set<Type> relems = right.isTop() ? types.getTypes() : right.elements;
        Set<Type> inferred = expression.getOperator().typeInference(types, lelems, relems);
        if (inferred.isEmpty())
            return PythonTypeSet.BOTTOM;
        return new PythonTypeSet(types, inferred);
    }

    @Override
    public PythonTypeSet evalTernaryExpression(
            TernaryExpression expression,
            PythonTypeSet left,
            PythonTypeSet middle,
            PythonTypeSet right,
            ProgramPoint pp,
            SemanticOracle oracle) {
        TypeSystem types = pp.getProgram().getTypes();
        Set<Type> lelems = left.isTop() ? types.getTypes() : left.elements;
        Set<Type> melems = middle.isTop() ? types.getTypes() : middle.elements;
        Set<Type> relems = right.isTop() ? types.getTypes() : right.elements;
        Set<Type> inferred = expression.getOperator().typeInference(types, lelems, melems, relems);
        if (inferred.isEmpty())
            return PythonTypeSet.BOTTOM;
        return new PythonTypeSet(types, inferred);
    }

    @Override
    public Satisfiability satisfiesBinaryExpression(
            BinaryExpression expression,
            PythonTypeSet left,
            PythonTypeSet right,
            ProgramPoint pp,
            SemanticOracle oracle) {
        TypeSystem types = pp.getProgram().getTypes();
        Set<Type> lelems = left.isTop() ? types.getTypes() : left.elements;
        Set<Type> relems = right.isTop() ? types.getTypes() : right.elements;
        BinaryOperator operator = expression.getOperator();
        if (operator == ComparisonEq.INSTANCE || operator == ComparisonNe.INSTANCE) {
            Set<Type> lfiltered = lelems.stream().filter(Type::isTypeTokenType).collect(Collectors.toSet());
            Set<Type> rfiltered = relems.stream().filter(Type::isTypeTokenType).collect(Collectors.toSet());

            if (lelems.size() != lfiltered.size() || relems.size() != rfiltered.size())
                // if there is at least one element that is not a type
                // token, than we cannot reason about it
                return Satisfiability.UNKNOWN;

            if (operator == ComparisonEq.INSTANCE) {
                if (lelems.size() == 1 && lelems.equals(relems))
                    // only one element, and it is the same
                    return Satisfiability.SATISFIED;
                else if (intersection(lelems, relems).isEmpty() && !typeTokensIntersect(lfiltered, rfiltered))
                    // no common elements, they cannot be equal
                    return Satisfiability.NOT_SATISFIED;
                else
                    // we don't know really
                    return Satisfiability.UNKNOWN;
            } else {
                if (intersection(lelems, relems).isEmpty() && !typeTokensIntersect(lfiltered, rfiltered))
                    // no common elements, they cannot be equal
                    return Satisfiability.SATISFIED;
                else if (lelems.size() == 1 && lelems.equals(relems))
                    // only one element, and it is the same
                    return Satisfiability.NOT_SATISFIED;
                else
                    // we don't know really
                    return Satisfiability.UNKNOWN;
            }
        } else if (operator == TypeCheck.INSTANCE) {
            if (evalBinaryExpression(expression.withOperator(TypeCast.INSTANCE), left, right, pp, oracle).isBottom())
                // no common types, the check will always fail
                return Satisfiability.NOT_SATISFIED;
            AtomicBoolean mightFail = new AtomicBoolean();
            Set<Type> set = types.cast(lelems, relems, mightFail);
            if (lelems.equals(set) && !mightFail.get())
                // if all the types stayed in 'set' then the there is no
                // execution that reach the expression with a type that cannot
                // be casted, and thus this is a tautology
                return Satisfiability.SATISFIED;

            // sometimes yes, sometimes no
            return Satisfiability.UNKNOWN;
        }

        return Satisfiability.UNKNOWN;
    }

    @Override
    public TypeEnvironment<PythonTypeSet> assumeUnaryExpression(
            TypeEnvironment<PythonTypeSet> environment,
            UnaryExpression expression,
            ProgramPoint src,
            ProgramPoint dest,
            SemanticOracle oracle)
            throws SemanticException {
        Satisfiability sat = satisfies(environment, expression, src, oracle);
        if (sat == Satisfiability.NOT_SATISFIED)
            return environment.bottom();
        if (sat == Satisfiability.SATISFIED)
            return environment;

        // we only support the negated type check
        if (!(expression.getOperator() instanceof LogicalNegation)
                || !(expression.getExpression() instanceof BinaryExpression)
                || !(((BinaryExpression) expression.getExpression()).getOperator() instanceof TypeCheck))
            return environment;

        Identifier id;
       PythonTypeSet eval;
        ValueExpression left = (ValueExpression) ((BinaryExpression) expression.getExpression()).getLeft();
        ValueExpression right = (ValueExpression) ((BinaryExpression) expression.getExpression()).getRight();
        if (left instanceof Identifier) {
            eval = eval(environment, right, src, oracle);
            id = (Identifier) left;
        } else if (right instanceof Identifier) {
            eval = eval(environment, left, src, oracle);
            id = (Identifier) right;
        } else
            return environment;

        TypeSystem types = src.getProgram().getTypes();
        Set<Type> elems = eval.isTop() ? types.getTypes() : eval.elements;
        if (elems.stream().anyMatch(Type::isTypeTokenType))
            // if there is no type token in the evaluation,
            // this is not a type condition and we cannot
            // assume anything
            return environment;

        // these are all types compatible with the type tokens
        Set<Type> okTypes = elems.stream()
                .filter(Type::isTypeTokenType)
                .map(Type::asTypeTokenType)
                .map(TypeTokenType::getTypes)
                .flatMap(Set::stream)
                .flatMap(t -> t.allInstances(types).stream())
                .collect(Collectors.toSet());

       PythonTypeSet starting = environment.getState(id);
        if (eval.isBottom() || starting.isBottom())
            return environment.bottom();
        // we keep only the ones that can be casted
        Set<Type> update = difference(starting.elements, okTypes);
        if (update.isEmpty())
            return environment.bottom();
        else
            return environment.putState(id, new PythonTypeSet(types, update));
    }

    @Override
    public TypeEnvironment<PythonTypeSet> assumeBinaryExpression(
            TypeEnvironment<PythonTypeSet> environment,
            BinaryExpression expression,
            ProgramPoint src,
            ProgramPoint dest,
            SemanticOracle oracle)
            throws SemanticException {
        Satisfiability sat = satisfies(environment, expression, src, oracle);
        if (sat == Satisfiability.NOT_SATISFIED)
            return environment.bottom();
        if (sat == Satisfiability.SATISFIED)
            return environment;

        Identifier id;
       PythonTypeSet eval;
        ValueExpression left = (ValueExpression) expression.getLeft();
        ValueExpression right = (ValueExpression) expression.getRight();
        if (left instanceof Identifier) {
            eval = eval(environment, right, src, oracle);
            id = (Identifier) left;
        } else if (right instanceof Identifier) {
            eval = eval(environment, left, src, oracle);
            id = (Identifier) right;
        } else
            return environment;

        TypeSystem types = src.getProgram().getTypes();
        Set<Type> elems = eval.isTop() ? types.getTypes() : eval.elements;
        Set<Type> tokens = elems.stream().filter(Type::isTypeTokenType).collect(Collectors.toSet());
        if (tokens.isEmpty())
            // if there is no type token in the evaluation,
            // this is not a type condition and we cannot
            // assume anything
            return environment;

        Set<Type> exactTypes = tokens.stream()
                .flatMap(t -> t.asTypeTokenType().getTypes().stream())
                .collect(Collectors.toSet());

        BinaryOperator operator = expression.getOperator();
       PythonTypeSet starting = environment.getState(id);
        if (eval.isBottom() || starting.isBottom())
            return environment.bottom();
        Set<Type> update = null;

        if (operator == ComparisonEq.INSTANCE)
            // we keep only the types allowed by the type tokens
            // that were already there
            update = intersection(starting.elements, exactTypes);
        else if (operator == ComparisonNe.INSTANCE)
            // we keep only the types that are not allowed by the type tokens
            update = difference(starting.elements, exactTypes);
        else if (operator == TypeCheck.INSTANCE)
            // we keep only the ones that can be casted
            update = types.cast(starting.elements, tokens);

        if (update == null)
            return environment;
        else if (update.isEmpty())
            return environment.bottom();
        else
            return environment.putState(id, new PythonTypeSet(types, update));
    }

    /**
     * Checks whether or not the two given set of type tokens intersects,
     * meaning that there exists at least one type token {@code t1} from
     * {@code lfiltered} and one type token {@code t2} from {@code rfiltered}
     * such that {@code t1.getTypes().intersects(t2.getTypes())}.<br>
     * <br>
     * Note that all types in both sets received as parameters are assumed to be
     * {@link TypeTokenType}s, hence no type check is performed before
     * converting them.
     *
     * @param lfiltered the first set of type tokens
     * @param rfiltered the second set of type tokens
     *
     * @return {@code true} if the sets of tokens intersect
     *
     * @throws NullPointerException if one of the types is not a
     *                                  {@link TypeTokenType} (this is due to
     *                                  the conversion)
     */
    static boolean typeTokensIntersect(
            Set<Type> lfiltered,
            Set<Type> rfiltered) {
        for (Type l : lfiltered)
            for (Type r : rfiltered)
                if (!intersection(l.asTypeTokenType().getTypes(), r.asTypeTokenType().getTypes()).isEmpty())
                    return true;

        return false;
    }

    @Override
    public PythonTypeSet evalTypeCast(
            BinaryExpression cast,
           PythonTypeSet left,
           PythonTypeSet right,
            ProgramPoint pp,
            SemanticOracle oracle) {
        TypeSystem types = pp.getProgram().getTypes();
        Set<Type> lelems = left.isTop() ? types.getTypes() : left.elements;
        Set<Type> relems = right.isTop() ? types.getTypes() : right.elements;
        Set<Type> inferred = cast.getOperator().typeInference(types, lelems, relems);
        if (inferred.isEmpty())
            return PythonTypeSet.BOTTOM;
        return new PythonTypeSet(types, inferred);
    }

    @Override
    public PythonTypeSet evalTypeConv(
            BinaryExpression conv,
           PythonTypeSet left,
           PythonTypeSet right,
            ProgramPoint pp,
            SemanticOracle oracle) {
        TypeSystem types = pp.getProgram().getTypes();
        Set<Type> lelems = left.isTop() ? types.getTypes() : left.elements;
        Set<Type> relems = right.isTop() ? types.getTypes() : right.elements;
        Set<Type> inferred = conv.getOperator().typeInference(types, lelems, relems);
        if (inferred.isEmpty())
            return PythonTypeSet.BOTTOM;
        return new PythonTypeSet(types, inferred);
    }

    @Override
    public PythonTypeSet top() {
        return PythonTypeSet.TOP;
    }

    @Override
    public PythonTypeSet bottom() {
        return PythonTypeSet.BOTTOM;
    }

}
