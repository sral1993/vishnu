//All file listings
SOURCE_FILES = main.c project_1_report.c memory.c data.c
PRE_FILES = main.i  project_1_report.i memory.i data.i
ASM_FILES = main.s  project_1_report.s memory.s data.s
OBJECT_FILES = main.o  project_1_report.o memory.o data.o


// providing path to the source and header folders/directories
vpath %.c ./source
vpath %.h ./header

