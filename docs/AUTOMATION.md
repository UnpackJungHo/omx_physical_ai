# Hook 기반 자동 상태 갱신

## 목적
- 에이전트가 특정 Phase 완료를 제안하면 사용자가 승인한 뒤 문서 상태를 자동 갱신한다.
- 자동 갱신은 `CLAUDE.md`, `docs/SESSION_NEXT.md`, `docs/STATUS.md`에 반영된다.

## 동작 흐름
1. 에이전트가 작업 완료를 판단한다.
2. 에이전트는 마지막 줄에 정확히 `[PHASE_COMPLETE_REQUEST:<PHASE>]`를 붙여 사용자 승인을 요청한다.
3. `Stop` hook이 이 마커를 감지하면 승인 대기 상태를 기록한다.
4. 사용자가 `승인: C 완료` 같은 문장으로 승인한다.
5. `UserPromptSubmit` hook이 승인을 감지하면 상태 파일과 문서를 자동 갱신한다.

## 권장 승인 문구
- `승인: C 완료`

## 허용되는 자연어 승인 예시
- `오케이 나도 C가 다 완료된거 같아`
- `C 완료 승인`
- `C는 끝난 것 같아, 승인할게`

## 관리 파일
- `.claude/settings.local.json`: Hook 등록
- `.claude/state/project_state.json`: Phase 상태와 승인 대기 상태
- `.claude/hooks/register_phase_completion.py`: 완료 제안 감지
- `.claude/hooks/process_phase_approval.py`: 사용자 승인 처리
- `.claude/hooks/phase_workflow.py`: 상태 갱신과 문서 렌더링 공용 로직

## 주의사항
- 자동 갱신은 선형 Phase 전이를 가정한다.
- 현재 구현은 `A -> B -> C -> D -> E` 순서를 기준으로 다음 Phase를 자동 활성화한다.
- 마커 없이 일반 대화만으로는 자동 갱신이 일어나지 않게 설계한다.
