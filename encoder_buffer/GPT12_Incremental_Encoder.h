#ifndef GPT12_INCREMENTAL_ENCODER_H_
#define GPT12_INCREMENTAL_ENCODER_H_

#include "Ifx_Types.h"

void initGpt12Encoder(void);
void updateGpt12Encoder(void);
float32 getGpt12SpeedRadS(void);
float32 getGpt12SpeedRpm(void);
sint32 getGpt12RawPosition(void);
sint32 getGpt12Direction(void);

#endif
