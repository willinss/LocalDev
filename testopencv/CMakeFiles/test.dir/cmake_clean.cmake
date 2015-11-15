FILE(REMOVE_RECURSE
  "CMakeFiles/test.dir/test.cpp.o"
  "CMakeFiles/test.dir/lib1.cpp.o"
  "CMakeFiles/test.dir/lib2.cpp.o"
  "test.pdb"
  "test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
