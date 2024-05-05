mkdir gen
..\..\tools\bubbler\bubbler.exe -t c -o .\gen\rovlink_core_c .\all\rovlink.bb
..\..\tools\bubbler\bubbler.exe -t py -decnum -signext=arith -o .\gen\rovlink_core_py .\all\rovlink.bb
