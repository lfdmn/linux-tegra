#include "../lucam.h"



static const unsigned char model_data_018a[] = {
1, 0 , 0 ,0 ,60,0,0,0,
1,0,
180,17,112,252,219,1,75,254,251,18,186,254,41,5,203,244,11,22,
2,0,
80,21,133,248,42,2,251,248,199,22,61,0,127,6,96,244,32,21,
5,0,
166,20,198,247,147,3,250,254,82,17,180,255,79,0,57,254,119,17,
0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // Lw160
   {
      0x018a,
      0xffff,
      model_data_018a,
      sizeof(model_data_018a),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


