package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.heap.HeapDomain;
import it.unive.lisa.analysis.type.TypeDomain;
import it.unive.lisa.analysis.value.ValueDomain;

public class ExtSimpleAbstractState<H extends HeapDomain<H>,
        V extends ValueDomain<V>,
        T extends TypeDomain<T>> extends SimpleAbstractState<H,V,T> {
    Imports imports;
    /**
     * Builds a new abstract state.
     *
     * @param heapState  the domain containing information regarding heap
     *                   structures
     * @param valueState the domain containing information regarding values of
     *                   program variables and concretized memory locations
     * @param typeState  the domain containing information regarding runtime
     *                   types of program variables and concretized memory
     *                   locations
     */
    public ExtSimpleAbstractState(H heapState, V valueState, T typeState) {
        super(heapState, valueState, typeState);
        this.imports = new Imports();
    }
}
