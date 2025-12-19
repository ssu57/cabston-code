# yolo_cam.py
import cv2
from ultralytics import YOLO

class YOLOBlurCam:
    def __init__(
        self,
        weight_paths,
        cam_index=0,
        width=640,
        height=480,
        imgsz=416,
        infer_interval=3
    ):
        self.models = [YOLO(p) for p in weight_paths]

        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            raise RuntimeError("카메라 오픈 실패")

        try:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        except Exception:
            pass

        self.imgsz = imgsz
        self.infer_interval = infer_interval
        self.frame_id = 0
        self.last_boxes = []

    def read_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return None
        return frame

    def process(
        self,
        frame,
        conf=0.25,
        blur=True,
        blur_ksize=31,
        margin_ratio=0.25
    ):
        self.frame_id += 1

        if self.frame_id % self.infer_interval == 0:
            self.last_boxes = []
            for model in self.models:
                r = model.predict(
                    frame,
                    imgsz=self.imgsz,
                    conf=conf,
                    verbose=False
                )[0]
                if r.boxes is None:
                    continue
                for box, cls in zip(r.boxes.xyxy, r.boxes.cls):
                    if model.names[int(cls)].lower() == "face":
                        self.last_boxes.append(box.cpu().numpy())

        if blur and self.last_boxes:
            k = blur_ksize if blur_ksize % 2 == 1 else blur_ksize + 1
            h, w = frame.shape[:2]
            for x1, y1, x2, y2 in self.last_boxes:
                bw, bh = x2 - x1, y2 - y1
                mx, my = bw * margin_ratio, bh * margin_ratio
                nx1 = max(0, int(x1 - mx))
                ny1 = max(0, int(y1 - my))
                nx2 = min(w, int(x2 + mx))
                ny2 = min(h, int(y2 + my))
                if nx2 <= nx1 or ny2 <= ny1:
                    continue
                roi = frame[ny1:ny2, nx1:nx2]
                if roi.size > 0:
                    frame[ny1:ny2, nx1:nx2] = cv2.GaussianBlur(
                        roi, (k, k), 0
                    )

        return frame, len(self.last_boxes)

    def release(self):
        self.cap.release()
##실행 코드 streamlit run app.py
