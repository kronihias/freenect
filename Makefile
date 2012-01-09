# Makefile
# (c) 2006 IOhannes m zm�lnig

# path to pd
## change this according to your setup!
PDROOT=/Users/matthias/Pd-0.42.5-extended/pd
LIBFREENECT=/home/matthias/libfreenect/include

# here we find the sources of pd (and evtl. the pd.lib)
PDSRCDIR=$(PDROOT)/src
PDLIBDIR=$(PDROOT)/bin

# this is the filename-extension
# people have to specify it at the cmdline: eg "make pd_linux"
EXTENSION=$(MAKECMDGOALS)

# if no filename-extension is supplied by the user
# try to guess one, based on what "uname" tells us
UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
  DEFAULTEXTENSION= pd_linux
else
  ifeq ($(UNAME),Darwin)
    DEFAULTEXTENSION= pd_darwin
  else
    ifeq (MINGW,$(findstring MINGW,$(UNAME)))
      DEFAULTEXTENSION= pd_nt
    else
      ifeq ($(UNAME),IRIX)
	UNAMEV := $(shell uname -R)
	ifeq (6.,$(findstring 6.,$(UNAMEV)))
	  DEFAULTEXTENSION= pd_irix6
	else
	  DEFAULTEXTENSION= pd_irix5
	endif
      else
	DEFAULTEXTENSION=help
      endif
    endif
  endif
endif

# if no extension is given, call "make" again with a guessed extension
auto:
	make $(DEFAULTEXTENSION)

# just a stupid fallback
help: 
	@echo "choose one command:  make pd_linux (linux), make pd_darwin (osX), make pd_irix5 (IRIX5), make pd_irix6 (IRIX6), make dll (MSVC), make pd_nt (MinWG)"

# delete old build files
clean:
	-rm -f *.dll *.pd_* *.o *.obj *~

# we want to compile all C-files we find in the current directory
SOURCES=$(sort $(filter %.c, $(wildcard *.c)))
# each C-files maps will become an external with the given filename-extension
TARGETS=$(SOURCES:.c=.$(EXTENSION))


# ----------------------- Linux -----------------------

pd_linux: $(TARGETS)

LINUXCFLAGS = -DPD -O2 -funroll-loops -fomit-frame-pointer -fPIC \
    -Wall -W -Wshadow -Wstrict-prototypes \
    -Wno-unused -Wno-parentheses -Wno-switch

LINUXLDFLAGS =  -export-dynamic -shared  -lc -lm

LINUXINCLUDE =  -I$(PDSRCDIR) -I$(LIBFREENECT)

%.pd_linux: %.c
	$(CC) $(LINUXLDFLAGS) $(LINUXCFLAGS) $(LINUXINCLUDE) -o $*.pd_linux $*.c -lfreenect
	strip --strip-unneeded $*.pd_linux



# ----------------------- Mac OSX -----------------------

pd_darwin: $(TARGETS)

DARWINCFLAGS = -DPD -O2 -Wall -W -Wshadow -Wstrict-prototypes \
    -Wno-unused -Wno-parentheses -Wno-switch -lfreenect

DARWININCLUDE = -I$(PDSRCDIR) -I$(LIBFREENECT)

DARWINLDFLAGS = -bundle -undefined suppress -flat_namespace -arch i386

%.pd_darwin: %.c
	$(CC) $(DARWINCFLAGS) $(DARWININCLUDE) $(DARWINLDFLAGS) -o $*.pd_darwin $*.c


# ----------------------- IRIX 5.x -----------------------
pd_irix5: $(TARGETS)

SGICFLAGS5 = -o32 -DPD -DSGI -O2

SGIINCLUDE =  -I$(PDSRCDIR)

SGILDFLAGS =  -elf -shared -rdata_shared

%.pd_irix5: %.c
	$(CC) $(SGICFLAGS5) $(SGIINCLUDE) -o $*.o -c $*.c
	$(LD) $(SGILDFLAGS) -o $*.pd_irix5 $*.o
	rm $*.o


# ----------------------- IRIX 6.x -----------------------
pd_irix6: $(TARGETS)

SGICFLAGS6 = -DPD -DSGI -n32 \
	-OPT:roundoff=3 -OPT:IEEE_arithmetic=3 -OPT:cray_ivdep=true \
	-Ofast=ip32

%.pd_irix6: %.c
	$(CC) $(SGICFLAGS6) $(SGIINCLUDE) -o $*.o -c $*.c
	$(LD) $(SGILDFLAGS) -o $*.pd_irix6 $*.o
	rm $*.o


# ----------------------- NT -----------------------
dll: $(TARGETS)

PDNTCFLAGS = /W3 /WX /DPD /DNT /D__WIN32__ /DMSW /nologo

VC="C:\Programme\Microsoft Visual Studio\Vc98"

PDNTINCLUDE = /I. /I$(PDROOT)\tcl\include /I$(PDSRCDIR)\src /I$(VC)\include

PDNTLDIR = $(VC)\lib

PDNTLIB = $(PDNTLDIR)\libc.lib \
	$(PDNTLDIR)\oldnames.lib \
	$(PDNTLDIR)\kernel32.lib \
	$(PDLIBDIR)\pd.lib 

%.dll: %.c
	cl $(PDNTCFLAGS) $(PDNTINCLUDE) /c $*.c
	link /dll /export:$*_setup $*.obj $(PDNTLIB)


pd_nt: $(TARGETS)

MINGWCFLAGS = -DPD -O2 -funroll-loops -fomit-frame-pointer \
    -Wall -W -Wshadow -Wstrict-prototypes -Werror \
    -Wno-unused -Wno-parentheses -Wno-switch -mms-bitfields

MINGWLDFLAGS =  -export_dynamic -shared -lm -lfreenect -lkernel32 -lcoldname -lcrtdll -lpd -L$(PDLIBDIR)

MINGWINCLUDE =  -I$(PDSRCDIR) -I$(LIBFREENECT)

%.pd_nt: %.c
	$(CC) $(MINGWLDFLAGS) $(MINGWCFLAGS) $(MINGWINCLUDE) -o $*.dll $*.c
