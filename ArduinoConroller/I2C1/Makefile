#
# Makefile:
#################################################################################
#################################################################################

DESTDIR=/usr
PREFIX=/local

DEBUG	= -g3
#DEBUG	= -O2
CC	= g++
INCLUDE	= -I$(DESTDIR)$(PREFIX)/include -I/home/pi/RobotV1/wiringPi/wiringPi
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -std=gnu++0x 

LDFLAGS	= -L$(DESTDIR)$(PREFIX)/lib
LIBS    = -lwiringPi -lwiringPiDev -lpthread -lm -lrt

# May not need to  alter anything below this line
###############################################################################

SRC	=	t1.cpp

OBJ	=	$(SRC:.cpp=.o)

all:		t1

t1:	$(OBJ)
	@echo [Link]
	@$(CC) -o $@ $(OBJ) $(LDFLAGS) $(LIBS)

.cpp.o:
	@echo [Compile] $<
	@$(CC) -c $(CFLAGS) $< -o $@

.PHONY:	clean
clean:
	@echo "[Clean]"
	@rm -f $(OBJ) t1 *~ core tags *.bak

.PHONY:	tags
tags:	$(SRC)
	@echo [ctags]
	@ctags $(SRC)

.PHONY:	install
install: t1
	@echo "[Install]"
	@cp t1		$(DESTDIR)$(PREFIX)/bin
	@chown root.root	$(DESTDIR)$(PREFIX)/bin/t1
	@chmod 4755		$(DESTDIR)$(PREFIX)/bin/t1
	@mkdir -p		$(DESTDIR)$(PREFIX)/man/man1
	@cp t1.1		$(DESTDIR)$(PREFIX)/man/man1

.PHONY:	install-deb
install-deb:	t1
	@echo "[Install: deb]"
	@install -m 0755 -d							~/wiringPi/debian/wiringPi/usr/bin
	@install -m 0755 t1							~/wiringPi/debian/wiringPi/usr/bin

.PHONY:	uninstall
uninstall:
	@echo "[UnInstall]"
	@rm -f $(DESTDIR)$(PREFIX)/bin/t1
	@rm -f $(DESTDIR)$(PREFIX)/man/man1/t1.1

.PHONY:	depend
depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
