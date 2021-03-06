This file is for use of the Project 1 in ECEN5013 at University of Colorado Boulder.

This file is written as part of Project1 in ECEN5013 to document the operations and syntax of Makefile

Please, go through the below mentioned guidelines.

Makefile Operations & Syntax

1. This Makefile supports three compilers i.e GCC(HOST Machine),ARM_LINUX-BeagleBone and ARM_NONE-FRDM-KL25Z.
   PLATFORM must be specified or else the default PLATFORM is the HOST Machine which is GCC.
   SYNTAX: make build PLATFORM=BBB
   
2. Makefile can generate preprocessed output.
   SYNTAX: make preprocess
   
3. Makefile can generate assembly files output. 
   SYNTAX: make asm-file
   
4. Makefile can generate single object file of the main file to be compiled.
   SYNTAX: make single_object_file
   
5. Makefile can also generate object files of all the files including the library files which are included
   SYNTAX: make compile-all

6. Makefile can generate the final executable file which is formed by linking all the files.
   SYNTAX: make build

7. Makefile can build a library and in our case it is the project_1_library. It generates both ordinary version and .tar version.
   SYNTAX: make build-lib

8. This make file can also upload the cross compiled object file to the beagle bone board.
   SYNTAX: make UPLOAD

9. This makefile is capable of cleaning all the directories. The directories should be cleaned before building the file again.
   SYNTAX: make clean

10. Once "make build" is used, inorder to build on other compiler "make clean" must be invoked.
