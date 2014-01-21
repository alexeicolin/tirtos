all:
# For Windows
ifeq ("$(SHELL)","sh.exe")
	@for /f %%d in ('dir /ad /b') do $(MAKE) all -C %%d
# For Linux
else
	@for dir in ./*; do                    \
             if [ -d $$dir ];              \
             then                          \
                  $(MAKE) all -C $$dir;    \
             fi                            \
         done
endif

clean:
# For Windows
ifeq ("$(SHELL)","sh.exe")
	@for /f %%d in ('dir /ad /b') do $(MAKE) clean -C %%d
# For Linux
else
	@for dir in ./*; do                    \
             if [ -d $$dir ];              \
             then                          \
                  $(MAKE) clean -C $$dir;  \
             fi                            \
         done
endif
