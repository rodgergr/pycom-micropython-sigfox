ifneq ($(MKENV_INCLUDED),1)
# We assume that mkenv is in the same directory as this file.
THIS_MAKEFILE = $(lastword $(MAKEFILE_LIST))
include $(dir $(THIS_MAKEFILE))mkenv.mk
endif

# This file expects that OBJ contains a list of all of the object files.
# The directory portion of each object file is used to locate the source
# and should not contain any ..'s but rather be relative to the top of the
# tree.
#
# So for example, py/map.c would have an object file name py/map.o
# The object files will go into the build directory and mantain the same
# directory structure as the source tree. So the final dependency will look
# like this:
#
# build/py/map.o: py/map.c
#
# We set vpath to point to the top of the tree so that the source files
# can be located. By following this scheme, it allows a single build rule
# to be used to compile all .c files.

vpath %.S . $(TOP)
$(BUILD)/%.o: %.S
	$(ECHO) "CC $<"
	$(Q)$(CC) $(CFLAGS) -c -o $@ $<

vpath %.s . $(TOP)
$(BUILD)/%.o: %.s
	$(ECHO) "AS $<"
	$(Q)$(AS) -o $@ $<

define compile_c
$(ECHO) "CC $<"
$(Q)$(CC) $(CFLAGS) -c -MD -o $@ $<
@# The following fixes the dependency file.
@# See http://make.paulandlesley.org/autodep.html for details.
@# Regex adjusted from the above to play better with Windows paths, etc.
@$(CP) $(@:.o=.d) $(@:.o=.P); \
  $(SED) -e 's/#.*//' -e 's/^.*:  *//' -e 's/ *\\$$//' \
      -e '/^$$/ d' -e 's/$$/ :/' < $(@:.o=.d) >> $(@:.o=.P); \
  $(RM) -f $(@:.o=.d)
endef

vpath %.c . $(TOP)
$(BUILD)/%.o: %.c
	$(call compile_c)

QSTR_GEN_EXTRA_CFLAGS += -DNO_QSTR
QSTR_GEN_EXTRA_CFLAGS += -I$(BUILD)/tmp

vpath %.c . $(TOP)

$(BUILD)/%.pp: %.c
	$(ECHO) "PreProcess $<"
	$(Q)$(CC) $(CFLAGS) -E -Wp,-C,-dD,-dI -o $@ $<

# The following rule uses | to create an order only prerequisite. Order only
# prerequisites only get built if they don't exist. They don't cause timestamp
# checking to be performed.
#
# We don't know which source files actually need the generated.h (since
# it is #included from str.h). The compiler generated dependencies will cause
# the right .o's to get recompiled if the generated.h file changes. Adding
# an order-only dependency to all of the .o's will cause the generated .h
# to get built before we try to compile any of them.
$(OBJ): | $(HEADER_BUILD)/qstrdefs.generated.h $(HEADER_BUILD)/mpversion.h

$(HEADER_BUILD)/qstr.i.last: $(SRC_QSTR) | $(HEADER_BUILD)/mpversion.h
	$(ECHO) "GEN $@"
	$(Q)$(CPP) $(QSTR_GEN_EXTRA_CFLAGS) $(CFLAGS) $(if $?,$?,$^) >$(HEADER_BUILD)/qstr.i.last;

$(HEADER_BUILD)/qstr.split: $(HEADER_BUILD)/qstr.i.last
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(PY_SRC)/makeqstrdefs.py split $(HEADER_BUILD)/qstr.i.last $(HEADER_BUILD)/qstr $(QSTR_DEFS_COLLECTED)
	$(Q)touch $@

$(QSTR_DEFS_COLLECTED): $(HEADER_BUILD)/qstr.split
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(PY_SRC)/makeqstrdefs.py cat $(HEADER_BUILD)/qstr.i.last $(HEADER_BUILD)/qstr $(QSTR_DEFS_COLLECTED)

# $(sort $(var)) removes duplicates
#
# The net effect of this, is it causes the objects to depend on the
# object directories (but only for existence), and the object directories
# will be created if they don't exist.
OBJ_DIRS = $(sort $(dir $(OBJ)))
$(OBJ): | $(OBJ_DIRS)
$(OBJ_DIRS):
	$(MKDIR) -p $@

$(HEADER_BUILD):
	$(MKDIR) -p $@

ifneq ($(FROZEN_DIR),)
$(BUILD)/frozen.c: $(wildcard $(FROZEN_DIR)/*) $(HEADER_BUILD) $(FROZEN_EXTRA_DEPS)
	$(ECHO) "GEN $@"
	$(Q)$(MAKE_FROZEN) $(FROZEN_DIR) > $@
endif

ifneq ($(FROZEN_MPY_DIR),)
OS_NAME := $(shell uname -s)
ifeq ($(VARIANT), PYBYTES)
ifeq ($(OS_NAME), Linux)
# make a list of all the .py files that need compiling and freezing
FROZEN_MPY_PY_FILES := $(shell find -L $(FROZEN_MPY_DIR)/Pybytes/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/Pybytes\///')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Common/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/Common\///')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Custom/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/Custom\///')
ifeq ($(BOARD), $(filter $(BOARD), GPY FIPY))
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/LTE/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/LTE\///')
endif
else
# make a list of all the .py files that need compiling and freezing
FROZEN_MPY_PY_FILES := $(shell find -L $(FROZEN_MPY_DIR)/Pybytes/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/Pybytes\//==')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Common/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/Common\//==')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Custom/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/Custom\//==')
ifeq ($(BOARD), $(filter $(BOARD), GPY FIPY))
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/LTE/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/LTE\//==')
endif
endif

FROZEN_MPY_MPY_FILES := $(addprefix $(BUILD)/frozen_mpy/,$(FROZEN_MPY_PY_FILES:.py=.mpy))

# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/Pybytes/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/Pybytes/%=%) $(MPY_CROSS_FLAGS) $^

# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/Common/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/Common/%=%) $(MPY_CROSS_FLAGS) $^
	
# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/Custom/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/Custom/%=%) $(MPY_CROSS_FLAGS) $^

ifeq ($(BOARD), $(filter $(BOARD), GPY FIPY))
# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/LTE/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/LTE/%=%) $(MPY_CROSS_FLAGS) $^
endif
	
endif
ifeq ($(VARIANT), BASE)
ifeq ($(OS_NAME), Linux)
# make a list of all the .py files that need compiling and freezing
FROZEN_MPY_PY_FILES := $(shell find -L $(FROZEN_MPY_DIR)/Base/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/Base\///')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Common/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/Common\///')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Custom/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/Custom\///')
ifeq ($(BOARD), $(filter $(BOARD), GPY FIPY))
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/LTE/ -type f -name '*.py' | $(SED) -e 's/$(FROZEN_MPY_DIR)\/LTE\///')
endif
else
# make a list of all the .py files that need compiling and freezing
FROZEN_MPY_PY_FILES := $(shell find -L $(FROZEN_MPY_DIR)/Base/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/Base\//==')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Common/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/Common\//==')
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/Custom/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/Custom\//==')
ifeq ($(BOARD), $(filter $(BOARD), GPY FIPY))
FROZEN_MPY_PY_FILES += $(shell find -L $(FROZEN_MPY_DIR)/LTE/ -type f -name '*.py' | $(SED) -e 's=^$(FROZEN_MPY_DIR)\/LTE\//==')
endif
endif

FROZEN_MPY_MPY_FILES := $(addprefix $(BUILD)/frozen_mpy/,$(FROZEN_MPY_PY_FILES:.py=.mpy))

# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/Base/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/Base/%=%) $(MPY_CROSS_FLAGS) $^

# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/Common/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/Common/%=%) $(MPY_CROSS_FLAGS) $^
	
# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/Custom/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/Custom/%=%) $(MPY_CROSS_FLAGS) $^

ifeq ($(BOARD), $(filter $(BOARD), GPY FIPY))	
# to build .mpy files from .py files
$(BUILD)/frozen_mpy/%.mpy: $(FROZEN_MPY_DIR)/LTE/%.py
	@$(ECHO) "MPY $<"
	$(Q)$(MKDIR) -p $(dir $@)
	$(Q)$(MPY_CROSS) -o $@ -s $(^:$(FROZEN_MPY_DIR)/LTE/%=%) $(MPY_CROSS_FLAGS) $^
endif

endif

# to build frozen_mpy.c from all .mpy files
$(BUILD)/frozen_mpy.c: $(FROZEN_MPY_MPY_FILES) $(BUILD)/genhdr/qstrdefs.generated.h
	@$(ECHO) "GEN $@"
	$(Q)$(MPY_TOOL) -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h $(FROZEN_MPY_MPY_FILES) > $@
endif

ifneq ($(PROG),)
# Build a standalone executable (unix does this)

all: $(PROG)

$(PROG): $(OBJ)
	$(ECHO) "LINK $@"
# Do not pass COPT here - it's *C* compiler optimizations. For example,
# we may want to compile using Thumb, but link with non-Thumb libc.
	$(Q)$(CC) -o $@ $^ $(LIB) $(LDFLAGS)
ifndef DEBUG
	$(Q)$(STRIP) $(STRIPFLAGS_EXTRA) $(PROG)
endif
	$(Q)$(SIZE) $$(find $(BUILD) -path "$(BUILD)/build/frozen*.o") $(PROG)

clean: clean-prog
clean-prog:
	$(RM) -f $(PROG)
	$(RM) -f $(PROG).map

.PHONY: clean-prog
endif

LIBMICROPYTHON = libmicropython.a

# We can execute extra commands after library creation using
# LIBMICROPYTHON_EXTRA_CMD. This may be needed e.g. to integrate
# with 3rd-party projects which don't have proper dependency
# tracking. Then LIBMICROPYTHON_EXTRA_CMD can e.g. touch some
# other file to cause needed effect, e.g. relinking with new lib.
lib $(LIBMICROPYTHON): $(OBJ)
	$(AR) rcs $(LIBMICROPYTHON) $^
	$(LIBMICROPYTHON_EXTRA_CMD)

clean:
	$(RM) -rf $(BUILD) $(CLEAN_EXTRA)
.PHONY: clean

# Clean every non-git file from FROZEN_DIR/FROZEN_MPY_DIR, but making a backup.
# We run rmdir below to avoid empty backup dir (it will silently fail if backup
# is non-empty).
clean-frozen:
	if [ -n "$(FROZEN_MPY_DIR)" ]; then \
	backup_dir=$(FROZEN_MPY_DIR).$$(date +%Y%m%dT%H%M%S); mkdir $$backup_dir; \
	cd $(FROZEN_MPY_DIR); git status --ignored -u all -s . | awk ' {print $$2}' \
	| xargs --no-run-if-empty cp --parents -t ../$$backup_dir; \
	rmdir ../$$backup_dir 2>/dev/null || true; \
	git clean -d -f .; \
	fi

	if [ -n "$(FROZEN_DIR)" ]; then \
	backup_dir=$(FROZEN_DIR).$$(date +%Y%m%dT%H%M%S); mkdir $$backup_dir; \
	cd $(FROZEN_DIR); git status --ignored -u all -s . | awk ' {print $$2}' \
	| xargs --no-run-if-empty cp --parents -t ../$$backup_dir; \
	rmdir ../$$backup_dir 2>/dev/null || true; \
	git clean -d -f .; \
	fi
.PHONY: clean-frozen

print-cfg:
	$(ECHO) "PY_SRC = $(PY_SRC)"
	$(ECHO) "BUILD  = $(BUILD)"
	$(ECHO) "OBJ    = $(OBJ)"
.PHONY: print-cfg

print-def:
	@$(ECHO) "The following defines are built into the $(CC) compiler"
	touch __empty__.c
	@$(CC) -E -Wp,-dM __empty__.c
	@$(RM) -f __empty__.c

-include $(OBJ:.o=.P)
