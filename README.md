These are the C/C++ libraries to interact with *qbmove* and *SoftHand*

If you want to use these libraries with [*qbmoveadmin*](https://github.com/qbrobotics/qbmoveadmin) or [*handadmin*](https://github.com/qbrobotics/handadmin) software, be sure to organize your folder as follows:

* your_workingcopy
    * qbAPI
    * qbmoveadmin
    * handadmin

### Compile the libraries

simply go into `/src` folder and type `make`. Depending on your OS you shold see a folder like this:

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