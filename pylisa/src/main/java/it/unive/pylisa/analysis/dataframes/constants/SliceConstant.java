package it.unive.pylisa.analysis.dataframes.constants;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.pylisa.cfg.type.PySliceType;

public class SliceConstant extends Constant {
    public SliceConstant(Slice slice, CodeLocation location) {
        super(PySliceType.INSTANCE, slice, location);
    }

    public SliceConstant(Integer start, Integer end, Integer skip, CodeLocation location) {
        super(PySliceType.INSTANCE, new Slice(start, end, skip), location);
    }

    public static class Slice {
        private Integer start, end, skip;
    
        public Slice(Integer start, Integer end, Integer skip) {
            this.start = start;
            this.end = end;
            this.skip = skip;
        }
    
        public Slice(Integer start, Integer end) {
            this.start = start;
            this.end = end;
            this.skip = null;
        }
        
        public Integer getStart() {
            return start;
        }
    
        public Integer getEnd() {
            return end;
        }
    
        public Integer getSkip() {
            return skip;
        }
    
        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof Slice))
                return false;
            Slice o = (Slice) obj;
            if (this.start == null) {
                if (o.start != null)
                    return false;
            } else if (!this.start.equals(o.start))
                return false;
    
            if (this.end == null) {
                if (o.end != null)
                    return false;
            } else if (!this.end.equals(o.end))
                return false;
    
            if (this.skip == null) {
                if (o.skip != null)
                    return false;
            } else if (!this.skip.equals(o.skip))
                return false;
            return true;
        }
    
        @Override
        public String toString() {
            return start + ":" + end + ":" + skip;
        }
    }
}
