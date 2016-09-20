# -*- MakeFile -*-



#This includes source.mk file which contains paths to the source and header directories and also some variables are declared 
include source.mk


#Defining Variables
#Variable HEADER_FILES is defined as all the contents of the folder header 
HEADER_FILES= ./header
#Default Platform is Host
CC= gcc
#Debugging information to the executable file is done by -g. -Wall turns on
#compiler warnings
CFLAG= -Wall -g -O0
#Providing route for the cross compiled executable file to the beaglebone board 
ROUTE = root@192.168.7.2:project


#Selecting the platform to which the code is to be compiled for
ifeq ($(PLATFORM),BBB)
#If Beagle Bone Black is selected as the platform on which the code is run on
CC=arm-linux-gnueabihf-gcc
else ifeq ($(PLATFORM),HOST)
#If Host machine is selected as the platform on which the code is to run on
CC = gcc 
#If Freedom freescale Board is selected as the platform on which the code is run on
else ifeq ($(PLATFORM), FRDM)
cc= arm-none-eabi-gcc
endif
#.PHONY is used to prevent clashes by explicitly telling Make that the target is not assosciated current directory



.PHONY: preprocess
#This generates the preprocessed output  i.e. the file after preprocessing is done
preprocess:$(PRE_FILES)
%.i: %.c
	$(CC) $(CFLAG) -I $(HEADER_FILES) -E $< -o $@ 
#$@ expands to the name of the outut file 
#$< expands to the name of the input file
 


.PHONY: asm-file
#This rule generates the files with .s extension i.e. by taking in preprocessed output it generates assembly level language code 
asm-file:$(ASM_FILES)
%.s: %.i
	$(CC) $(CFLAG) -S $< -o $@



.PHONY: single_object_file
#This rule generates a single object file main.o in our case
single_object_file:$(OBJECT_FILES)
%.o: %.s
	$(CC) $(CFLAG) -I $(HEADER_FILES) -c $< -o $@



.PHONY:  compile_all
#This rule compiles all the files including those in the source directory and creates object files
compile-all:$(OBJECT_FILES)
	$(CC) $(CFLAG) -I $(HEADER_FILES) -c $<



.PHONY: build
#This rule also links all the files which are in .o format i.e. object files 
build:$(OBJECT_FILES)
	$(CC) $(CFLAG) -I $(HEADER_FILES) $(OBJECT_FILES) -o result

	size -A result  #It lists out the sizes of various sextions as well as the total size  
	objdump -a $(OBJECT_FILES)      #It lists out all the object files and their respective formats



.PHONY:Build-lib
#This rule builds a library, which is named libproject.a in our case
build-lib: $(SOURCE_FILES)
	ar rU project_1_library.a ./source/*.c
	ranlib project_1_library.a
#This  part  creates the tar version of libproject which is an archive file format  
	tar -cvf project_1_library.tar ./source/*.c   
#This rule sends the executable file to the BeagleBoneBlack Board



#This rule uploads the cross compiled file to the board  and its route is provided by ROUTE variable in thsi makefile
UPLOAD: result
	scp result $(ROUTE)


.PHONY: clean
#This rule removes all the files from the directory which have the extensions .s, .i, .o 
clean:
	rm -rf *.i *.s *.o Output


