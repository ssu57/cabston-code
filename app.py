# app.py
import streamlit as st
import cv2
import time
import os
import secrets
import string
import pyzipper
from datetime import datetime
from yolo_cam import YOLOBlurCam

# ================================
# 암호 키 자동 생성
# ================================
def generate_key(length=8):
    chars = string.ascii_letters + string.digits
    return ''.join(secrets.choice(chars) for _ in range(length))

# ================================
# 기본 설정
# ================================
st.set_page_config(
    page_title="실시간 프라이버시 보호 시스템",
    layout="wide"
)
st.title("🎥 실시간 비식별화 기반 프라이버시 보호 시스템")

CONF_FIXED = 0.25

WEIGHTS = [
    r"C:\Users\원수민\Desktop\privacy_ui\best1.pt",
    r"C:\Users\원수민\Desktop\privacy_ui\best2.pt",
    r"C:\Users\원수민\Desktop\privacy_ui\best3.pt",
]

# ================================
# 저장 경로
# ================================
BASE_DIR = "records"
BLUR_DIR = os.path.join(BASE_DIR, "blur")
ORG_DIR = os.path.join(BASE_DIR, "original_enc")
os.makedirs(BLUR_DIR, exist_ok=True)
os.makedirs(ORG_DIR, exist_ok=True)

# ================================
# 세션 상태
# ================================
defaults = {
    "recording": False,
    "record_buffer": [],
    "failsafe": False,
    "save_blur": False,
    "save_original": False,
    "last_enc_key": None,
    "fps_avg": 0.0,
    "prev_time": time.time(),
}
for k, v in defaults.items():
    if k not in st.session_state:
        st.session_state[k] = v

# ================================
# 사이드바
# ================================
st.sidebar.header("🎬 녹화 / 저장 제어")

if st.sidebar.button("▶ 녹화 시작 / 중지"):
    st.session_state.recording = not st.session_state.recording

if st.sidebar.button("💾 비식별 저장"):
    st.session_state.save_blur = True

if st.sidebar.button("🔐 원본 저장 (암호화)"):
    st.session_state.save_original = True

if st.sidebar.button("⛔ 즉시 차단"):
    st.session_state.failsafe = True

# ================================
# 카메라 (세션당 1회)
# ================================
if "cam" not in st.session_state:
    st.session_state.cam = YOLOBlurCam(WEIGHTS)

cam = st.session_state.cam

status_bar = st.empty()
col1, col2 = st.columns([3, 1])
frame_box = col1.empty()
info_box = col2.empty()

# ================================
# FPS 계산
# ================================
now = time.time()
dt = now - st.session_state.prev_time
st.session_state.prev_time = now
fps = 1.0 / dt if dt > 0 else 0.0
st.session_state.fps_avg = (
    0.9 * st.session_state.fps_avg + 0.1 * fps
    if st.session_state.fps_avg > 0 else fps
)

# ================================
# 프레임 처리 (1프레임)
# ================================
frame = cam.read_frame()
if frame is None:
    status_bar.markdown("## 🔴 상태등 : ERROR (카메라 입력 없음)")
else:
    if st.session_state.recording:
        st.session_state.record_buffer.append(frame.copy())

    if st.session_state.failsafe:
        frame_view = cv2.GaussianBlur(frame, (31, 31), 0)
        infer_state = "중단됨 (Fail-Safe)"
        status_code = "🔴 ERROR"
    else:
        frame_view, _ = cam.process(
            frame.copy(),
            conf=CONF_FIXED,
            blur=True,
            blur_ksize=31,
            margin_ratio=0.25
        )
        infer_state = "정상"
        status_code = "🟢 NORMAL"

    status_bar.markdown(f"## 상태등 : {status_code}")
    frame_box.image(cv2.cvtColor(frame_view, cv2.COLOR_BGR2RGB), channels="RGB")

    with info_box.container():
        st.markdown("### 📊 시스템 상태")
        st.metric("FPS", f"{st.session_state.fps_avg:.1f}")
        st.metric("추론 상태", infer_state)
        st.metric("하트비트", "정상")
        st.metric("상태 코드", status_code)

        st.markdown("---")
        st.metric("녹화 상태", "ON" if st.session_state.recording else "OFF")
        st.metric("녹화 프레임 수", len(st.session_state.record_buffer))

        if st.session_state.last_enc_key:
            st.warning(f"🔑 최근 암호 키: {st.session_state.last_enc_key}")

# ================================
# 💾 비식별 저장
# ================================
if st.session_state.save_blur:
    if st.session_state.record_buffer:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(BLUR_DIR, f"blur_{ts}.mp4")
        h, w = st.session_state.record_buffer[0].shape[:2]

        vw = cv2.VideoWriter(
            path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            20,
            (w, h)
        )

        for f in st.session_state.record_buffer:
            bf, _ = cam.process(
                f.copy(),
                conf=CONF_FIXED,
                blur=True,
                blur_ksize=31,
                margin_ratio=0.25
            )
            vw.write(bf)
        vw.release()

    st.session_state.save_blur = False

# ================================
# 🔐 원본 저장 (AES 암호화)
# ================================
if st.session_state.save_original:
    if st.session_state.record_buffer:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        mp4_path = os.path.join(ORG_DIR, f"original_{ts}.mp4")

        h, w = st.session_state.record_buffer[0].shape[:2]
        vw = cv2.VideoWriter(
            mp4_path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            20,
            (w, h)
        )
        for f in st.session_state.record_buffer:
            vw.write(f)
        vw.release()

        enc_key = generate_key()
        st.session_state.last_enc_key = enc_key

        zip_path = mp4_path.replace(".mp4", ".zip")
        key_path = zip_path.replace(".zip", ".key.txt")

        with pyzipper.AESZipFile(
            zip_path,
            "w",
            compression=pyzipper.ZIP_DEFLATED,
            encryption=pyzipper.WZ_AES
        ) as zf:
            zf.setpassword(enc_key.encode())
            zf.write(mp4_path, arcname=os.path.basename(mp4_path))

        with open(key_path, "w") as f:
            f.write(f"Encryption Key: {enc_key}\n")

        os.remove(mp4_path)

    st.session_state.save_original = False

# ================================
# 다음 프레임 요청
# ================================
time.sleep(0.03)
st.experimental_rerun()

##코드실행 streamlit run app.py