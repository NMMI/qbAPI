These are the C/C++ libraries to interact with *qbmove* and *SoftHand*

If you want to use these libraries with
[*qbmoveadmin*](https://github.com/qbrobotics/qbmoveadmin) or
[*handadmin*](https://github.com/qbrobotics/handadmin) software,
be sure to organize your folder as follows:

* your_workingcopy
    * qbAPI
    * qbmoveadmin
    * handadmin

### Compile the libraries

simply go into `/src` folder and type "make". You now should have a
folder organized like follows:

* qbAPI
    * lib
        * libqbmove_comm.a
    * objs
    * src
    * license.txt
    * README.txt

> The generated `libqbmove_comm.a` can be linked to your libraries and is
> used by *qbmoveadmin* and *handadmin*