# 3d_illusion

순서


1. 얼굴인식
   1. 사진 가져오기
   2. 딥러닝을 통해서 얼굴 인식
   3. 얼굴 시선 각도 구해서 정면 얼굴 추출
2. 눈인식
   1. 눈 위치 정보 구하기
3. 거리 구하기
   1. Focal length 구하기
   2. 눈 위치 정보를 통해서 거리 구하기
4. 언리얼 적용하기
   1. 언리얼에서 파이썬 코드 실행해서 데이터 전송 (거리, 눈 위치)
   2. 3D 환경 만들기
   3. 얼굴 위치에 따라서 카메라 위치 업데이트 하기