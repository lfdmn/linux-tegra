#include "../lucam.h"



static const unsigned char model_data_04f7[] = {
1, 0 , 0 ,0 ,80,0,0,0,
1,0,
72,49,190,226,250,251,12,252,162,25,83,250,121,248,88,250,47,29,
2,0,
51,35,0,240,205,252,52,251,153,25,52,251,205,252,154,253,153,21,
3,0,
86,50,66,231,106,246,91,245,2,37,166,245,58,244,21,3,176,24,
5,0,
51,35,0,240,205,252,52,251,153,25,52,251,205,252,154,253,153,21,

2,0,0,0, 4,0,0,0,
2,0,0,0,

0,0,0,0};


struct _ModelDta2 ModelDtaList[] = 
{

   // Infinity1-2B
   {
      0x04f7,
      0xffff,
      model_data_04f7,
      sizeof(model_data_04f7)/sizeof(model_data_04f7[0]),
   },
};

int ModelDtaListCount = sizeof(ModelDtaList) / sizeof(ModelDtaList[0]);


