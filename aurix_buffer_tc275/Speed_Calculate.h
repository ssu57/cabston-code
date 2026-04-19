#ifndef SPEED_CALCULATE_H_
#define SPEED_CALCULATE_H_

#include "Ifx_Types.h"

/*
 * 엔코더 기본 설정
 * - ENCODER_PPR x 4배 계수 = 출력축 1회전당 엔코더 카운트 수
 * - 현재 프로젝트는 최근 8개 RPM 샘플 평균만 사용한다.
 */
#define ENCODER_PPR               330U
#define ENCODER_X4                4U
#define COUNTS_PER_REV            (ENCODER_PPR * ENCODER_X4)
#define SPEED_BUFFER_FIXED_COUNT  8U

typedef struct
{
    float32 rawRpm;        /* 최신 RPM 샘플 */
    float32 avgRpm;        /* 최근 8개 RPM 샘플 평균 */
    uint32  rawTick100us;  /* 최신 RPM 샘플 계산에 사용된 시간차(100us) */
    uint32  avgTick100us;  /* 최근 8개 시간차의 평균(100us) */
    boolean valid;
} SpeedValue;

void SpeedCalc_Init(void);
boolean SpeedCalc_Get(SpeedValue *out);

#endif /* SPEED_CALCULATE_H_ */
