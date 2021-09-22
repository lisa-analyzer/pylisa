#TODO list

We need to define a language to specify library behaviors. The grammar should look to something like

<pre>
< library_name > {
  (< method_signature> {
        < method_specification >
  }) *

  (< class_name > {
        < field_specification > *
       (< method_signature> {
               < method_specification >
        }) *
  })*
}
</pre>

**NOTE** It is not clear at all what should go into the specifications!

Then we should parse each single file like this one. Each file should lead to:
* a PythonUnit (see PyToCFG.getWarningsPythonUnit) that returns NativeCFGs for the various methods that implements the semantics of the given specification
  * this will need to rely on a PluggableStatement like StupidConstruct (for now, this is total nonsense)
* register the unit through PyLibraryType.addUnit
* register a PyLibraryType with the name of the parsed library 



