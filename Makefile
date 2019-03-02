CC          = g++
SRCDIR      = src
BUILDDIR    = build
BINDIR      = bin

TARGET  = slerp
INC     = isometry

CFLAGS = -Wall -std=c++11 -Wno-int-in-bool-context -Wno-misleading-indentation -I include
LDLIBS = -lglut -lGLU -lGL

$(BINDIR)/$(TARGET): $(BUILDDIR)/$(TARGET).o $(BUILDDIR)/$(INC).o
	@ mkdir -p $(BINDIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BUILDDIR)/$(TARGET).o: $(SRCDIR)/$(TARGET).cpp include/$(INC).hpp
	@ mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILDDIR)/$(INC).o: $(SRCDIR)/$(INC).cpp include/$(INC).hpp
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: clean

clean:
	-rm -r $(BUILDDIR) $(BINDIR)

