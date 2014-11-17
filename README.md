These are the C/C++ libraries to interact with *qbmove* and *SoftHand*

If you want to use these libraries with [*qbmoveadmin*](https://github.com/qbrobotics/qbmoveadmin) or [*handadmin*](https://github.com/qbrobotics/handadmin) software, be sure to organize your folder as follows:

* your_workingcopy
    * qbAPI
    * qbmoveadmin
    * handadmin

## Install the compiler

It is not mandatory to use the makefile to compile the library, but it is suggested,
so here follows how to configure your computer to do so.

### Unix
You should have both gcc/g++ and make installed

### MacOSX
Download XCode from Mac App Store, this will install gcc/g++ and make utility

### Windows
Download [MinGW](http://www.mingw.org) and install it. Open MinGW Installation
Manager, from the left panel select basic setup, then from the right panel select
mingw32-base and mingw32-gcc-g++, then click on `Installation -> Apply Changes`.
This will install the gcc/g++ compiler. To use it from the command line you need
to provide to windows the binary path to the executable. Go to System Properties
and click on Environment Variables. In the System Variable windows, look for `path`,
select it and click `edit`. Go to the end of the Variable Value field, add a `;`
separator and add the path to the binari folder for gcc (usually it is in C:\MinGW\bin).

Now you need the Make utility. Download it from
[here](http://gnuwin32.sourceforge.net/packages/make.htm). Follow the installation
instruction. In the end you will need to add the binary path to the Environment
Variables. To do that follow the same guide as above. (Usually the binary fodler
for the make utility is in C:\Program Files (x86)\GnuWin32\bin).

>NOTE: if you have the CMD already opened when performing the installation,
>you probably will need to reopen a new one to be able to use the utilities.


### Compile the libraries

simply go into `/src` folder and type `make`. Depending on your OS you shold
see a folder like this:

* qbAPI
    * lib_win
        * libqbmove_comm.a
    * objs_win
    * src
    * license.txt
    * README.txt

or

* qbAPI
    * lib_unix
        * libqbmove_comm.a
    * objs_unix
    * src
    * license.txt
    * README.txt

> The generated `libqbmove_comm.a` can be linked to your libraries and is
> used by *qbmoveadmin* and *handadmin*
