# 3d_illusion

<앱 안에서 동작 - Android기기>
-Android studio로 개발?
1. camera2 API로 웹캠 입력정보 받기
2. 카메라 입력 데이터 서버 컴퓨터로 전송

<서버 컴퓨터에서 동작 - Aws 컴퓨터>
1. Opencv로 카메라 입력 데이터 실시간 Tracking 처리
- 최초 시작 좌표를 기준으로 상대적인 움직임을 계산

2. 실시간 Tracking 데이터에서 유의미한 데이터 추출
- 눈의 xy좌표 , 얼굴 회전각 , 거리감?

3. 추출 데이터 unreal 엔진으로 연결
4. unreal 엔진에서 추출 데이터 반영한 출력 이미지 처리
5. 처리된 이미지 클라이언트 기기로 전송

<앱 안에서 동작>
3. 서버에서 전송 받은 실시간 이미지 출력
