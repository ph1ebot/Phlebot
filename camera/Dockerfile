FROM dtam_ml_ros:latest

COPY jetcam /usr/jetcam
RUN cd /usr/jetcam && \
    python3 setup.py install

# Install atexit, cv2, threading, traitlets
RUN pip3 install traitlets scikit-build
