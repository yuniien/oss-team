# 5.blob_filtering - By: Janghun Hyeon - Fri Nov 22 2024
from pyb import UART
import sensor, image, time

# 1단계: Camera 스트리밍
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()


# 2단계: Blob 만들기
class Blob:
    def __init__(self, x, y, w, h):
        self.x =x
        self.y =y
        self.w =w
        self.h =h

    def rect(self):
        return (self.x,self.y, self.w, self.h)

    def area(self):
        return self.w * self.h

# 3단계: 특정 색상 법위로 blob 감지
threshold_orange = (200, 250, 150, 230, 30, 80) # 빨간색 임계값 설정

def find_blobs(img, threshold):
    width = img.width()
    height = img.height()
    blobs = []

    for y in range(0,height,5): # 간격 (5 pixel)을 두고 스캔
        for x in range(0,width,5):
            pixel = img.get_pixel(x, y)
            r, g, b = pixel

            if (r>=threshold_orange[0] and r<=threshold_orange[1]) and (g>=threshold_orange[2] and g<=threshold_orange[3]) and (b>=threshold_orange[4] and b<=threshold_orange[5]):
                blobs.append(Blob(x,y,5,5)) # Blob 추가
    return blobs


# 4단계: Blob 병합
def merge_blobs(blobs, distance_threshold=50):
    merged_blobs = []

    while blobs:
        current_blob = blobs.pop(0)
        merged = False

        for merged_blob in merged_blobs:
            dx = abs(current_blob.x - merged_blob.x)
            dy = abs(current_blob.y - merged_blob.y)

            if dx <= distance_threshold and dy <= distance_threshold:
                x1 = min(current_blob.x, merged_blob.x)
                y1 = min(current_blob.y, merged_blob.y)
                x2 = max(current_blob.x + current_blob.w, merged_blob.x + merged_blob.w)
                y2 = max(current_blob.y + current_blob.h, merged_blob.y + merged_blob.h)

                merged_blob.x = x1
                merged_blob.y = y1
                merged_blob.w = x2-x1
                merged_blob.h = y2-y1

                merged = True
                break

        if not merged:
            merged_blobs.append(current_blob)

    return merged_blobs


# 5단계: Blob 크기 필터링
MIN_AREA_THRESHOLD = 1000 # 필터링 임계값 설정
uart=UART(3, 115200, timeout_char=200)

a=[]

while(True):
    clock.tick()
    img = sensor.snapshot()
    blobs = find_blobs(img, threshold_orange)

    merged_blobs = merge_blobs(blobs)

    for blob in merged_blobs:
        if Blob.area(blob)>MIN_AREA_THRESHOLD:
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.x + blob.w // 2, blob.y + blob.h //2)

            # blob 정보 출력
            print("Blob [면적: %d, 중심 좌표: (%d, %d), 가로: %d, 세로: %d]" %(blob.area(), blob.x + blob.w//2, blob.y + blob.h//2, blob.w, blob.h))
            a.append(blob.x + blob.w//2)
            a.append(blob.y + blob.h//2)
            uart.write("%d, %d" %(blob.x + blob.w//2, blob.y + blob.h//2))
            uart.write("%d, %d" %(a[0],a[1]))
            uart.write("bye")
            time.sleep_ms(1000)
            print('send')

    #print(clock.fps())

