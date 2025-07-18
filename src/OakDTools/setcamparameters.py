import depthai as dai
import cv2

STEP_SIZE = 8
EXP_STEP = 10 # us
ISO_STEP = 50
LENS_STEP = 3
WB_STEP = 200

lensMin = 0
lensMax = 255

expMin = 1
expMax = 33000

sensMin = 100
sensMax = 1600

wbMin = 1000
wbMax = 12000

def clamp(num, v0, v1):
    return max(v0, min(num, v1))

class SetCamParameters():
  def __init__(self, device,lensPos=150,expTime=350,sensIso=800,wbManual=4000):
      self.lensPos=lensPos
      self.expTime=expTime
      self.sensIso=sensIso
      self.wbManual=wbManual
      self.device=device
      self.controlQueue = self.device.getInputQueue('control')
      self.configQueue = self.device.getInputQueue('config')
      self.previewQueue = self.device.getOutputQueue('preview')

  def manual_tuning(self):
        while True:
          previewFrames = self.previewQueue.tryGetAll()
          for previewFrame in previewFrames:
              cv2.imshow('preview', previewFrame.getData().reshape(previewFrame.getHeight(), previewFrame.getWidth(), 3))
          key = cv2.waitKey(1)
          if key == ord('q'):
              break
          elif key == ord('t'):
              print("Autofocus trigger (and disable continuous)")
              ctrl = dai.CameraControl()
              ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
              ctrl.setAutoFocusTrigger()
              self.controlQueue.send(ctrl)
          elif key == ord('f'):
              print("Autofocus enable, continuous")
              ctrl = dai.CameraControl()
              ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
              self.controlQueue.send(ctrl)
          elif key == ord('e'):
              print("Autoexposure enable")
              ctrl = dai.CameraControl()
              ctrl.setAutoExposureEnable()
              self.controlQueue.send(ctrl)
          elif key == ord('b'):
              print("Auto white-balance enable")
              ctrl = dai.CameraControl()
              ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
              self.controlQueue.send(ctrl)
          elif key in [ord(','), ord('.')]:
              if key == ord(','): lensPos -= LENS_STEP
              if key == ord('.'): lensPos += LENS_STEP
              lensPos = clamp(self.lensPos, lensMin, lensMax)
              print("Setting manual focus, lens position: ", lensPos)
              ctrl = dai.CameraControl()
              ctrl.setManualFocus(lensPos)
              self.controlQueue.send(ctrl)
          elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
              if key == ord('i'): expTime -= EXP_STEP
              if key == ord('o'): expTime += EXP_STEP
              if key == ord('k'): sensIso -= ISO_STEP
              if key == ord('l'): sensIso += ISO_STEP
              expTime = clamp(self.expTime, expMin, expMax)
              sensIso = clamp(self.sensIso, sensMin, sensMax)
              print("Setting manual exposure, time: ", expTime, "iso: ", sensIso)
              ctrl = dai.CameraControl()
              ctrl.setManualExposure(expTime, sensIso)
              self.controlQueue.send(ctrl)
          elif key in [ord('['), ord(']')]:
              if key == ord('['): wbManual -= WB_STEP
              if key == ord(']'): wbManual += WB_STEP
              wbManual = clamp(self.wbManual, wbMin, wbMax)
              print("Setting manual white balance, temperature: ", wbManual, "K")
              ctrl = dai.CameraControl()
              ctrl.setManualWhiteBalance(wbManual)
              self.controlQueue.send(ctrl)



