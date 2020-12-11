(file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:19:58.046 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:19:58.050 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:19:58.162 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:19:58.162 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:19:58.169 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:19:58.170 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:19:58.171 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:19:58.171 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:19:58.172 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:19:58.174 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:19:58.175 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:19:58.176 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:19:58.176 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:19:58.179 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:19:58.179 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:19:58.179 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:19:58.181 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:19:58.182 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:19:58.182 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:19:58.182 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:19:58.183 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:19:58.184 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:19:58.186 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:19:58.187 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:19:58.187 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:19:58.187 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:19:58.188 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:19:58.188 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:19:58.189 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:19:58.189 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:19:58.189 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:19:58.189 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:19:58.189 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:19:58.195 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 32ms 924500ns]
[INFO ] 2020-12-11 18:19:58.195 [main] PyToCFG - class it.unive.pylisa.antlr.Python3Parser$File_inputContext
[INFO ] 2020-12-11 18:19:58.198 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:22:38.904 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:22:38.908 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:22:39.047 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:22:39.048 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:22:39.063 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:22:39.065 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:22:39.065 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:22:39.066 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:22:39.067 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:22:39.070 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:22:39.070 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:22:39.071 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:22:39.071 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:22:39.073 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:22:39.073 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:22:39.073 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:22:39.074 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:22:39.075 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:22:39.076 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:22:39.076 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:22:39.076 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:22:39.077 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:22:39.078 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:22:39.078 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:22:39.079 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:22:39.080 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:22:39.080 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:22:39.080 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:22:39.081 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:22:39.081 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:22:39.081 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:22:39.081 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:22:39.081 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:22:39.082 [main] PyToCFG - 0
[INFO ] 2020-12-11 18:22:39.087 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 41ms 42900ns]
[INFO ] 2020-12-11 18:22:39.089 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:24:07.762 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:24:07.766 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:24:07.883 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:24:07.883 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:24:07.893 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:24:07.894 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:24:07.894 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:07.895 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:07.896 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:24:07.902 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:24:07.902 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:07.903 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:24:07.903 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:24:07.906 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:07.906 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:24:07.907 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:07.907 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:24:07.909 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:24:07.910 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:24:07.910 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:07.910 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:24:07.911 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:24:07.911 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:07.912 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:07.912 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:24:07.913 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:24:07.914 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:24:07.914 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:07.915 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:24:07.915 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:24:07.915 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:07.915 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:07.915 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:24:08.121 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:24:08.125 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 242ms 193400ns]
[INFO ] 2020-12-11 18:24:08.127 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:24:41.069 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:24:41.073 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:24:41.167 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:24:41.168 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:24:41.183 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:24:41.184 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:24:41.185 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:41.186 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:41.188 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:24:41.192 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:24:41.193 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:41.193 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:24:41.193 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:24:41.195 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:41.195 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:24:41.195 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:41.196 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:24:41.197 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:24:41.197 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:24:41.197 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:41.198 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:24:41.198 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:24:41.199 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:41.200 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:41.200 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:24:41.201 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:24:41.202 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:24:41.202 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:41.202 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:24:41.203 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:24:41.203 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:24:41.203 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:24:41.203 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:24:41.206 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:24:41.211 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 43ms 170500ns]
[INFO ] 2020-12-11 18:24:41.212 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:26:51.807 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:26:51.811 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:26:51.907 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:26:51.909 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:26:51.926 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:26:51.927 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:26:51.928 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:26:51.928 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:26:51.929 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:26:51.931 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:26:51.931 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:26:51.931 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:26:51.931 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:26:51.933 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:26:51.933 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:26:51.933 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:26:51.933 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:26:51.934 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:26:51.934 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:26:51.934 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:26:51.935 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:26:51.935 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:26:51.935 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:26:51.936 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:26:51.937 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:26:51.939 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:26:51.943 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 36ms 265600ns]
[INFO ] 2020-12-11 18:26:51.945 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:28:15.850 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:28:15.853 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:28:15.963 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:28:15.964 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:28:15.973 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:28:15.974 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:28:15.974 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:28:15.976 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:28:15.980 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:28:15.981 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:28:15.983 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:28:15.983 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:28:15.984 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:28:15.984 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:28:15.984 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:28:15.985 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:28:15.985 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:28:15.986 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:28:15.986 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:28:15.986 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:28:15.986 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:28:15.987 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:28:15.987 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:28:15.987 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:28:15.987 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:28:15.987 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:28:15.987 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:28:15.988 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:28:15.990 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:28:15.995 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 31ms 694500ns]
[INFO ] 2020-12-11 18:28:15.997 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:31:50.767 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:31:50.772 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:31:50.878 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:31:50.879 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:31:50.904 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:31:50.905 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:31:50.905 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:31:50.906 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:31:50.910 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.911 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:31:50.913 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.914 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:31:50.922 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:31:50.922 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.922 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:31:50.922 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.923 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:31:50.923 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:31:50.923 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:31:50.923 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:31:50.923 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:31:50.924 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:31:50.925 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:31:50.925 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:31:50.925 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:31:50.925 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.925 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:31:50.925 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.926 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:31:50.926 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:31:50.926 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:31:50.926 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:31:50.926 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:31:50.926 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:31:50.927 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:31:50.946 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:31:50.951 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 73ms 253200ns]
[INFO ] 2020-12-11 18:31:50.953 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:33:33.786 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:33:33.789 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:33:33.870 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:33:33.871 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:33:33.876 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:33:33.877 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:33:33.877 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:33:33.877 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:33:33.880 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:33:33.880 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:33:33.881 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:33:33.881 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:33:33.885 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:33:33.885 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:33:33.885 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:33:33.885 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:33:33.886 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:33:33.887 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:33:33.887 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:33:33.887 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:33:33.888 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:33:33.889 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:33:33.906 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:33:33.911 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 40ms 93500ns]
[INFO ] 2020-12-11 18:33:33.912 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
[INFO ] 2020-12-11 18:39:52.072 [main] PyToCFG - PyToCFG setup...
[INFO ] 2020-12-11 18:39:52.076 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2020-12-11 18:39:52.203 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2020-12-11 18:39:52.203 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2020-12-11 18:39:52.210 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:39:52.211 [main] PyToCFG - if7>2:                 pos=3   else:                 pos=444
[INFO ] 2020-12-11 18:39:52.211 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.211 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:39:52.214 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.214 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:39:52.215 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.216 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:39:52.219 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.219 [main] PyToCFG - MAGGIORE
[INFO ] 2020-12-11 18:39:52.219 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.219 [main] PyToCFG - 7
[INFO ] 2020-12-11 18:39:52.219 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.219 [main] PyToCFG - 2
[INFO ] 2020-12-11 18:39:52.220 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:39:52.220 [main] PyToCFG - pos=3  
[INFO ] 2020-12-11 18:39:52.220 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.220 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:39:52.220 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:39:52.221 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.221 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.221 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:39:52.222 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.222 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:39:52.222 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - 3
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - Sono nel suite
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - pos=44
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:39:52.223 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - NAME:
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - pos
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - Sono nel test
[INFO ] 2020-12-11 18:39:52.224 [main] PyToCFG - NUMBER:
[INFO ] 2020-12-11 18:39:52.225 [main] PyToCFG - 4
[INFO ] 2020-12-11 18:39:52.241 [main] PyToCFG - Done
[INFO ] 2020-12-11 18:39:52.246 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 43ms 258400ns]
[INFO ] 2020-12-11 18:39:52.248 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (compound_stmt (if_stmt if (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 7)))))))))) (comp_op >) (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 2))))))))))))))) : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 3))))))))))))))))))   ))  ) else : (suite                    (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom pos)))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (arith_expr (term (factor (power (atom_expr (atom 4)))))))))))))))))) 4)) 4)))) 4)))) s = 4)
