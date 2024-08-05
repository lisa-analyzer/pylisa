package it.unive.pylisa.program.annotations;

import it.unive.lisa.program.annotations.values.AnnotationValue;
import it.unive.lisa.program.cfg.statement.Expression;

public class TypeHintAnnotation implements AnnotationValue {
  Expression e;

  public TypeHintAnnotation(Expression e) {
    this.e = e;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((e == null) ? 0 : e.hashCode());
    return result;
  }

  @Override
  public String toString() {
    return e.toString();
  }

  @Override
  public boolean equals(
      Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    TypeHintAnnotation other = (TypeHintAnnotation) obj;
    if (e == null) {
      if (other.e != null)
        return false;
    } else if (!e.equals(other.e))
      return false;
    return true;
  }

  @Override
  public int compareTo(AnnotationValue o) {
    if (!(o instanceof TypeHintAnnotation))
      return getClass().getName().compareTo(o.getClass().getName());
    TypeHintAnnotation other = (TypeHintAnnotation) o;
    return e.compareTo(other.e);
  }
}
