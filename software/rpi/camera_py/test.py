import cv2
from picamera2 import Picamera2
from libcamera import controls
import os

picam2 = Picamera2()

#HDRをオンにする
os.system("v4l2-ctl --set-ctrl wide_dynamic_range=1 -d /dev/v4l-subdev0")
print("Setting HDR to ON")

picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()
#カメラを連続オートフォーカスモードにする
#picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous})
picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "AfSpeed": controls.AfSpeedEnum.Fast})

while True:
  im = picam2.capture_array()
  cv2.imshow("Camera", im)
 
  key = cv2.waitKey(1)
  # Escキーを入力されたら画面を閉じる
  if key == 27:
    break

picam2.stop()
cv2.destroyAllWindows()