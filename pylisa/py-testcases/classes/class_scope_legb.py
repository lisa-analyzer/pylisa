# Tests LEGB scope isolation:
# - class body fields must NOT leak into method scope as bare names
# - local method variables must shadow nothing (they are truly local)
# - module-level names are accessible as global fallback

MODULE_VAR = 42

class Foo:
    bar = 5          # class-level field

    def method_local(self):
        bar = 99     # local — MUST be VariableRef("bar"), not module ref
        return bar   # reads local bar

    def method_global(self):
        return MODULE_VAR  # reads module-level name (G in LEGB)

    def method_self(self):
        self.bar = 10    # instance attribute write (PyAccessInstanceGlobal)
        return self.bar  # instance attribute read (AttributeAccess)
