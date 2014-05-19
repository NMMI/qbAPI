#ifndef QBMOVELIBRARY_H
#define QBMOVELIBRARY_H

class qbmove {
private:
    int num_of_qb;
    int* ids;

public:
    qbmove(int number_of_qbmove, int* id_list, char* port_name);
    ~qbmove();
};



#endif

/* END OF FILE */