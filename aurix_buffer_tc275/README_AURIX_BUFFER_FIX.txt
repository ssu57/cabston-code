AURIX_BUFFER_FIX 프로젝트 요약

1) 메인 루프
- 5 ms 주기 유지
- GPT12 ISR 100 us 유지

2) RPM 계산 방식
- 엔코더 위치 차이와 시간 차이로 최신 RPM 샘플 계산
- 최근 8개 RPM 샘플을 링버퍼에 저장
- 직전 누적합을 유지하면서 8개 평균 RPM 계산

3) 듀티 시퀀스
- 100% -> 0% -> 25% -> 50% -> 75% -> 100%
- 각 구간 4초 유지
- 마지막 100% 이후 계속 100% 유지

4) CAN 송신 신호
- 0x100: RAW_PHASE4_TICK100US, AVG_PHASE4_TICK100US, RAW_RPM, AVG_RPM
- 0x101: ACTIVE_BUFFER_COUNT, BEST_BUFFER_COUNT, RPM_STDDEV, RPM_P2P, DUTY_PERCENT, STATE

5) 고정 버퍼 정보
- ACTIVE_BUFFER_COUNT = 8
- BEST_BUFFER_COUNT = 8
