//------------------------------------------------------------------     INCLUDE

#include <string.h>


#include "qbmove_communications.h"
#include "qbmovelibrary.h"



qbmove::qbmove(int number_of_qbmove, int* id_list, char* port_name) {
    num_of_qb = number_of_qbmove;
    ids = new int[num_of_qb];
    memcpy(ids, id_list, sizeof(int)*num_of_qb);
}

qbmove::~qbmove(){
    delete ids;
}

/* END OF FILE */