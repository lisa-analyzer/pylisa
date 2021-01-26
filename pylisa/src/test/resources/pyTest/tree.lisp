[INFO ] 2021-01-26 13:18:35.946 [main] PyToCFG - PyToCFG setup...
[INFO ] 2021-01-26 13:18:35.954 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2021-01-26 13:18:36.193 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 3)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom b))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 6)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom c))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom "ciao")))))))))))))))))))))   )) (stmt (compound_stmt (while_stmt while (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))) (comp_op <=) (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 10)))))))))))))))))) : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (add (term (mul (factor (power (atom_expr (atom contatore)))))) + (arith_expr (minus (term (mul (factor (power (atom_expr (atom 1)))))))))))))))))))))))   ))  )))) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 56)))))))))))))))))))))  ))  ))))      )
[INFO ] 2021-01-26 13:18:36.201 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2021-01-26 13:18:36.208 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2021-01-26 13:18:36.213 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.214 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.214 [main] PyToCFG - a
[INFO ] 2021-01-26 13:18:36.217 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.217 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:18:36.217 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:18:36.221 [main] PyToCFG - a
[INFO ] 2021-01-26 13:18:36.221 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:18:36.222 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:18:36.225 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:18:36.225 [main] PyToCFG - small
[INFO ] 2021-01-26 13:18:36.225 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:18:36.225 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:18:36.225 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:18:36.226 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.226 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.226 [main] PyToCFG - b
[INFO ] 2021-01-26 13:18:36.227 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.228 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:18:36.228 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:18:36.229 [main] PyToCFG - b
[INFO ] 2021-01-26 13:18:36.229 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:18:36.229 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.230 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.230 [main] PyToCFG - small
[INFO ] 2021-01-26 13:18:36.230 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.230 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:18:36.231 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.233 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.233 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.233 [main] PyToCFG - b
[INFO ] 2021-01-26 13:18:36.233 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.234 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:18:36.234 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:18:36.234 [main] PyToCFG - b
[INFO ] 2021-01-26 13:18:36.235 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:18:36.235 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.235 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.236 [main] PyToCFG - small
[INFO ] 2021-01-26 13:18:36.236 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.236 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:18:36.237 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:18:36.237 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.237 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.237 [main] PyToCFG - c
[INFO ] 2021-01-26 13:18:36.238 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.238 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:18:36.244 [main] PyToCFG - c
[INFO ] 2021-01-26 13:18:36.244 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:18:36.245 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:18:36.246 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:18:36.246 [main] PyToCFG - small
[INFO ] 2021-01-26 13:18:36.247 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:18:36.247 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:18:36.247 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:18:36.247 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.247 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.248 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:18:36.248 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.249 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:18:36.249 [main] PyToCFG - 10
[INFO ] 2021-01-26 13:18:36.255 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.256 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.256 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:18:36.257 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.257 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.257 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:18:36.257 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.258 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:18:36.258 [main] PyToCFG - 1
[INFO ] 2021-01-26 13:18:36.258 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:18:36.264 [main] PyToCFG - +(contatore, 1)
[INFO ] 2021-01-26 13:18:36.264 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:18:36.265 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:18:36.265 [main] PyToCFG - small
[INFO ] 2021-01-26 13:18:36.265 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:18:36.265 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:18:36.265 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:18:36.266 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.266 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:18:36.266 [main] PyToCFG - a
[INFO ] 2021-01-26 13:18:36.267 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:18:36.269 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:18:36.272 [main] PyToCFG - 56
[INFO ] 2021-01-26 13:18:36.272 [main] PyToCFG - a
[INFO ] 2021-01-26 13:18:36.272 [main] PyToCFG - 56
[INFO ] 2021-01-26 13:18:36.272 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:18:36.273 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:18:36.273 [main] PyToCFG - small
[INFO ] 2021-01-26 13:18:36.273 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:18:36.273 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:18:36.274 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:18:36.322 [main] PyToCFG - Done
[INFO ] 2021-01-26 13:18:36.328 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 124ms 407500ns]
[INFO ] 2021-01-26 13:19:45.671 [main] PyToCFG - PyToCFG setup...
[INFO ] 2021-01-26 13:19:45.674 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2021-01-26 13:19:45.900 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 3)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom b))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 6)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom c))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom "ciao")))))))))))))))))))))   )) (stmt (compound_stmt (while_stmt while (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))) (comp_op <=) (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 10)))))))))))))))))) : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (add (term (mul (factor (power (atom_expr (atom contatore)))))) + (arith_expr (minus (term (mul (factor (power (atom_expr (atom 1)))))))))))))))))))))))   ))  ) else : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom print))))))))))))))))))))))) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom "hello")))))))))))))))))))))   ))  )))) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 56)))))))))))))))))))))  ))  ))))      )
[INFO ] 2021-01-26 13:19:45.904 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2021-01-26 13:19:45.907 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2021-01-26 13:19:45.911 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.911 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.911 [main] PyToCFG - a
[INFO ] 2021-01-26 13:19:45.914 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.914 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:19:45.914 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:19:45.915 [main] PyToCFG - a
[INFO ] 2021-01-26 13:19:45.916 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:19:45.917 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:19:45.918 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:19:45.919 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.919 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:19:45.919 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.919 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:19:45.919 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.919 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.920 [main] PyToCFG - b
[INFO ] 2021-01-26 13:19:45.920 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.920 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:19:45.920 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:19:45.920 [main] PyToCFG - b
[INFO ] 2021-01-26 13:19:45.920 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:19:45.921 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.921 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.921 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.921 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.921 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.921 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.922 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.922 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.922 [main] PyToCFG - b
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - b
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.923 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.924 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.924 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.924 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.924 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:19:45.924 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.924 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.925 [main] PyToCFG - c
[INFO ] 2021-01-26 13:19:45.925 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.925 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:19:45.926 [main] PyToCFG - c
[INFO ] 2021-01-26 13:19:45.926 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:19:45.927 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:19:45.927 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:19:45.927 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.927 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:19:45.927 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.927 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:19:45.928 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.928 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.928 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:19:45.928 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.928 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:19:45.929 [main] PyToCFG - 10
[INFO ] 2021-01-26 13:19:45.931 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.932 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.932 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:19:45.932 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.933 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:19:45.933 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:19:45.933 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:19:45.933 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:19:45.933 [main] PyToCFG - 1
[INFO ] 2021-01-26 13:19:45.933 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:19:45.942 [main] PyToCFG - +(contatore, 1)
[INFO ] 2021-01-26 13:19:45.943 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:19:45.943 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:19:45.943 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.943 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:19:45.944 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.944 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:19:45.945 [main] PyToCFG - null
[INFO ] 2021-01-26 13:19:45.946 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.946 [main] PyToCFG - null
[INFO ] 2021-01-26 13:19:45.946 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.946 [main] PyToCFG - null
[INFO ] 2021-01-26 13:19:45.946 [main] PyToCFG - null
[INFO ] 2021-01-26 13:19:45.947 [main] PyToCFG - small
[INFO ] 2021-01-26 13:19:45.947 [main] PyToCFG - null
[INFO ] 2021-01-26 13:19:45.947 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:19:45.947 [main] PyToCFG - null
[INFO ] 2021-01-26 13:21:36.684 [main] PyToCFG - PyToCFG setup...
[INFO ] 2021-01-26 13:21:36.686 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2021-01-26 13:21:36.903 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 3)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom b))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 6)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom c))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom "ciao")))))))))))))))))))))   )) (stmt (compound_stmt (while_stmt while (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))) (comp_op <=) (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 10)))))))))))))))))) : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (add (term (mul (factor (power (atom_expr (atom contatore)))))) + (arith_expr (minus (term (mul (factor (power (atom_expr (atom 1)))))))))))))))))))))))   ))  ) else : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom print))))))))))))))))))))))) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom "hello")))))))))))))))))))))   ))  )))) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 56)))))))))))))))))))))  ))  ))))      )
[INFO ] 2021-01-26 13:21:36.906 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2021-01-26 13:21:36.909 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2021-01-26 13:21:36.913 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.914 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.914 [main] PyToCFG - a
[INFO ] 2021-01-26 13:21:36.917 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.917 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:21:36.917 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:21:36.919 [main] PyToCFG - a
[INFO ] 2021-01-26 13:21:36.919 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:21:36.920 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:21:36.921 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:21:36.922 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.922 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:21:36.922 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.922 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:21:36.922 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.923 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.923 [main] PyToCFG - b
[INFO ] 2021-01-26 13:21:36.923 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.923 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:21:36.923 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:21:36.924 [main] PyToCFG - b
[INFO ] 2021-01-26 13:21:36.924 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:21:36.925 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.925 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.927 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.927 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.927 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.928 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.928 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.929 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.931 [main] PyToCFG - b
[INFO ] 2021-01-26 13:21:36.932 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.932 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:21:36.932 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:21:36.933 [main] PyToCFG - b
[INFO ] 2021-01-26 13:21:36.933 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:21:36.933 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.933 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.934 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.934 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.934 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.934 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:21:36.935 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.935 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.935 [main] PyToCFG - c
[INFO ] 2021-01-26 13:21:36.936 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.936 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:21:36.937 [main] PyToCFG - c
[INFO ] 2021-01-26 13:21:36.938 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:21:36.938 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:21:36.938 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:21:36.939 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.939 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:21:36.940 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.940 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:21:36.940 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.941 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.942 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:21:36.942 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.942 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:21:36.943 [main] PyToCFG - 10
[INFO ] 2021-01-26 13:21:36.947 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.947 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.947 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:21:36.948 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.948 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:21:36.948 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:21:36.948 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:21:36.948 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:21:36.949 [main] PyToCFG - 1
[INFO ] 2021-01-26 13:21:36.949 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:21:36.955 [main] PyToCFG - +(contatore, 1)
[INFO ] 2021-01-26 13:21:36.955 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:21:36.955 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:21:36.955 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - else
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - null
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.956 [main] PyToCFG - null
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - null
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - null
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - small
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - null
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:21:36.957 [main] PyToCFG - null
[INFO ] 2021-01-26 13:22:14.648 [main] PyToCFG - PyToCFG setup...
[INFO ] 2021-01-26 13:22:14.651 [main] PyToCFG - Reading file... src/test/resources/pyTest/py1.py
[INFO ] 2021-01-26 13:22:14.866 [main] PyToCFG - (file_input \r\n (stmt (compound_stmt (funcdef def _repr_info (parameters ( (typedargslist (tfpdef self)) )) : (suite             (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 3)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom b))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 6)))))))))))))))))))))   )) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom c))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom "ciao")))))))))))))))))))))   )) (stmt (compound_stmt (while_stmt while (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))) (comp_op <=) (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 10)))))))))))))))))) : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom contatore))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (add (term (mul (factor (power (atom_expr (atom contatore)))))) + (arith_expr (minus (term (mul (factor (power (atom_expr (atom 1)))))))))))))))))))))))   ))  ) else : (suite                     (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 2)))))))))))))))))))))   ))  )))) (stmt (simple_stmt (small_stmt (expr_stmt (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom a))))))))))))))))))) = (testlist_star_expr (test (or_test (and_test (not_test (comparison (expr (xor_expr (and_expr (shift_expr (left_shift (arith_expr (minus (term (mul (factor (power (atom_expr (atom 56)))))))))))))))))))))  ))  ))))      )
[INFO ] 2021-01-26 13:22:14.874 [main] PyToCFG - Parsing stmt lists... [start]
[INFO ] 2021-01-26 13:22:14.877 [main] PyToCFG - Parsing stmt lists...: 1/1
[INFO ] 2021-01-26 13:22:14.880 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.881 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.881 [main] PyToCFG - a
[INFO ] 2021-01-26 13:22:14.884 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.885 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.885 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:22:14.886 [main] PyToCFG - a
[INFO ] 2021-01-26 13:22:14.887 [main] PyToCFG - 3
[INFO ] 2021-01-26 13:22:14.888 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:22:14.890 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:22:14.890 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.890 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:22:14.890 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - a = 3
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - b
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:22:14.891 [main] PyToCFG - b
[INFO ] 2021-01-26 13:22:14.892 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:22:14.892 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.892 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.892 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.892 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.893 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.893 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.894 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.894 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.894 [main] PyToCFG - b
[INFO ] 2021-01-26 13:22:14.895 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.895 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.895 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:22:14.895 [main] PyToCFG - b
[INFO ] 2021-01-26 13:22:14.896 [main] PyToCFG - 6
[INFO ] 2021-01-26 13:22:14.897 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.897 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.898 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.898 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.898 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.898 [main] PyToCFG - b = 6
[INFO ] 2021-01-26 13:22:14.898 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.899 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.899 [main] PyToCFG - c
[INFO ] 2021-01-26 13:22:14.900 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.900 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:22:14.901 [main] PyToCFG - c
[INFO ] 2021-01-26 13:22:14.901 [main] PyToCFG - "ciao"
[INFO ] 2021-01-26 13:22:14.902 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:22:14.902 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:22:14.902 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.902 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:22:14.902 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.902 [main] PyToCFG - c = "ciao"
[INFO ] 2021-01-26 13:22:14.903 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.903 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.903 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:22:14.903 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.904 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.904 [main] PyToCFG - 10
[INFO ] 2021-01-26 13:22:14.920 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.920 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.921 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:22:14.921 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.922 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.922 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:22:14.922 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.922 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.922 [main] PyToCFG - 1
[INFO ] 2021-01-26 13:22:14.922 [main] PyToCFG - contatore
[INFO ] 2021-01-26 13:22:14.931 [main] PyToCFG - +(contatore, 1)
[INFO ] 2021-01-26 13:22:14.931 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:22:14.931 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:22:14.931 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.931 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:22:14.932 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.932 [main] PyToCFG - contatore = +(contatore, 1)
[INFO ] 2021-01-26 13:22:14.933 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.934 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.934 [main] PyToCFG - a
[INFO ] 2021-01-26 13:22:14.950 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.950 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.950 [main] PyToCFG - 2
[INFO ] 2021-01-26 13:22:14.950 [main] PyToCFG - a
[INFO ] 2021-01-26 13:22:14.950 [main] PyToCFG - 2
[INFO ] 2021-01-26 13:22:14.950 [main] PyToCFG - a = 2
[INFO ] 2021-01-26 13:22:14.951 [main] PyToCFG - a = 2
[INFO ] 2021-01-26 13:22:14.951 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.953 [main] PyToCFG - a = 2
[INFO ] 2021-01-26 13:22:14.953 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.954 [main] PyToCFG - a = 2
[INFO ] 2021-01-26 13:22:14.955 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.955 [main] PyToCFG - NAME:
[INFO ] 2021-01-26 13:22:14.955 [main] PyToCFG - a
[INFO ] 2021-01-26 13:22:14.956 [main] PyToCFG - dentro il term
[INFO ] 2021-01-26 13:22:14.956 [main] PyToCFG - NUMBER:
[INFO ] 2021-01-26 13:22:14.957 [main] PyToCFG - 56
[INFO ] 2021-01-26 13:22:14.957 [main] PyToCFG - a
[INFO ] 2021-01-26 13:22:14.957 [main] PyToCFG - 56
[INFO ] 2021-01-26 13:22:14.957 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:22:14.958 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:22:14.958 [main] PyToCFG - small
[INFO ] 2021-01-26 13:22:14.958 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:22:14.958 [main] PyToCFG - sono in simple
[INFO ] 2021-01-26 13:22:14.958 [main] PyToCFG - a = 56
[INFO ] 2021-01-26 13:22:15.022 [main] PyToCFG - Done
[INFO ] 2021-01-26 13:22:15.028 [main] PyToCFG - Parsing stmt lists... [stop] [1 Global stmt in 150ms 823100ns]
